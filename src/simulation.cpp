#include "simulation.h"

#include <cmath>
#include <cstdlib>
#include <omp.h>

namespace {

void normalizeSpeed(const Margin &margin, float &vx, float &vy) {
  const float speed = std::sqrt(vx * vx + vy * vy);
  if (speed == 0.0f) {
    vx = static_cast<float>(margin.minSpeed);
    vy = 0.0f;
    return;
  }

  if (speed < margin.minSpeed) {
    vx = (vx / speed) * margin.minSpeed;
    vy = (vy / speed) * margin.minSpeed;
  }
  if (speed > margin.maxSpeed) {
    vx = (vx / speed) * margin.maxSpeed;
    vy = (vy / speed) * margin.maxSpeed;
  }
}

} // namespace

void initParallelSimulation(Grid &grid, BoidSoA &boids, int size) {
  placeBoids(grid.getMargin(), boids, size);
  grid.buildGrid(boids);
}

void placeBoids(const Margin &margin, BoidSoA &boids, int size) {
  boids.init(size);
  for (int i = 0; i < size; ++i) {
    float x = static_cast<float>(
        rand() % (margin.rightMargin - margin.leftMargin + 1) +
        margin.leftMargin);
    float y = static_cast<float>(
        rand() % (margin.bottomMargin - margin.topMargin + 1) +
        margin.topMargin);
    float vx = static_cast<float>((rand() % (margin.maxSpeed * 2 + 1)) -
                                  margin.maxSpeed);
    float vy = static_cast<float>((rand() % (margin.maxSpeed * 2 + 1)) -
                                  margin.maxSpeed);
    boids.push_back(i, x, y, vx, vy);
  }
}
void runParallelSoA(Grid &grid, BoidSoA &boids) {
#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
    float xpos_avg = 0.0f;
    float ypos_avg = 0.0f;
    float xvel_avg = 0.0f;
    float yvel_avg = 0.0f;
    float closeDx = 0.0f;
    float closeDy = 0.0f;
    int neighboring_boids = 0;

    grid.findNeighbors(boids, i, xpos_avg, ypos_avg, xvel_avg, yvel_avg,
                       neighboring_boids, closeDx, closeDy);
    grid.applyRulesToBoid(boids, i, xpos_avg, ypos_avg, xvel_avg, yvel_avg,
                          closeDx, closeDy, neighboring_boids);
  }
  for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
    grid.move(boids, i);
  }
}

void runSequential(const Margin &margin, BoidSoA &boids) {
  const int boidCount = static_cast<int>(boids.size());
  const double visualRangeSquared = margin.visualRange * margin.visualRange;
  const float protectedRangeSquared =
      margin.protectedRange * margin.protectedRange;

  for (int i = 0; i < boidCount; ++i) {
    float xposAvg = 0.0f;
    float yposAvg = 0.0f;
    float xvelAvg = 0.0f;
    float yvelAvg = 0.0f;
    float closeDx = 0.0f;
    float closeDy = 0.0f;
    int neighboringBoids = 0;

    const float boidX = boids.x[i];
    const float boidY = boids.y[i];

    for (int j = 0; j < boidCount; ++j) {
      if (j == i) {
        continue;
      }

      const float dx = boidX - boids.x[j];
      const float dy = boidY - boids.y[j];

      if (std::abs(dx) < margin.visualRange &&
          std::abs(dy) < margin.visualRange) {
        const double squaredDistance =
            static_cast<double>(dx) * dx + static_cast<double>(dy) * dy;

        if (squaredDistance < protectedRangeSquared) {
          closeDx += dx;
          closeDy += dy;
        } else if (squaredDistance < visualRangeSquared) {
          xposAvg += boids.x[j];
          yposAvg += boids.y[j];
          xvelAvg += boids.vx[j];
          yvelAvg += boids.vy[j];
          ++neighboringBoids;
        }
      }
    }

    const float currentVx = boids.vx[i];
    const float currentVy = boids.vy[i];
    float nextVx = currentVx;
    float nextVy = currentVy;

    if (neighboringBoids > 0) {
      xposAvg /= neighboringBoids;
      yposAvg /= neighboringBoids;
      xvelAvg /= neighboringBoids;
      yvelAvg /= neighboringBoids;

      nextVx = currentVx + (xposAvg - boidX) * margin.centeringFactor +
               (xvelAvg - currentVx) * margin.matchingFactor;
      nextVy = currentVy + (yposAvg - boidY) * margin.centeringFactor +
               (yvelAvg - currentVy) * margin.matchingFactor;
    }

    nextVx += closeDx * margin.avoidFactor;
    nextVy += closeDy * margin.avoidFactor;

    if (boidX < margin.leftMargin) {
      nextVx += margin.turnFactor;
    }
    if (boidX > margin.rightMargin) {
      nextVx -= margin.turnFactor;
    }
    if (boidY > margin.bottomMargin) {
      nextVy -= margin.turnFactor;
    }
    if (boidY < margin.topMargin) {
      nextVy += margin.turnFactor;
    }

    normalizeSpeed(margin, nextVx, nextVy);

    boids.vx[i] = nextVx;
    boids.vy[i] = nextVy;
    boids.x[i] = boidX + nextVx;
    boids.y[i] = boidY + nextVy;
  }
}
