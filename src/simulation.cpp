#include "simulation.h"

#include <cmath>
#include <cstdlib>
#include <omp.h>

namespace {

void normalizeSpeed(const Options &options, float &vx, float &vy) {
  const float speed = std::sqrt(vx * vx + vy * vy);
  if (speed == 0.0f) {
    vx = static_cast<float>(options.minSpeed);
    vy = 0.0f;
    return;
  }

  if (speed < options.minSpeed) {
    vx = (vx / speed) * options.minSpeed;
    vy = (vy / speed) * options.minSpeed;
  }
  if (speed > options.maxSpeed) {
    vx = (vx / speed) * options.maxSpeed;
    vy = (vy / speed) * options.maxSpeed;
  }
}

} // namespace

void initParallelSimulation(Grid &grid, BoidSoA &boids, int size) {
  placeBoids(grid.getMargin(), boids, size);
  grid.buildGrid(boids);
}

void placeBoids(const Options &margin, BoidSoA &boids, int size) {
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

void runParallelSoASingleBoid(Grid &grid, BoidSoA &boids, int i) {
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

void runParallelSoA(Grid &grid, BoidSoA &boids,
                    ScheudulingStrategyEnum schedulingStrategy) {
  switch (schedulingStrategy) {
  case ScheudulingStrategyEnum::Dynamic:
#pragma omp parallel for schedule(dynamic, 4096)
    for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
      runParallelSoASingleBoid(grid, boids, i);
    }
    // code block
    break;
  case ScheudulingStrategyEnum::Guided:
#pragma omp parallel for schedule(guided, 4096)
    for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
      runParallelSoASingleBoid(grid, boids, i);
    }
    // code block
    break;
  case ScheudulingStrategyEnum::Static:
#pragma omp parallel for schedule(static, 4096)
    for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
      runParallelSoASingleBoid(grid, boids, i);
    }
    break;
    // code block
  }
  for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
    grid.move(boids, i);
  }
}

void runSequential(const Options &options, BoidSoA &boids) {
  const int boidCount = static_cast<int>(boids.size());
  const double visualRangeSquared = options.visualRange * options.visualRange;
  const float protectedRangeSquared =
      options.protectedRange * options.protectedRange;

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

      if (std::abs(dx) < options.visualRange &&
          std::abs(dy) < options.visualRange) {
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

      nextVx = currentVx + (xposAvg - boidX) * options.centeringFactor +
               (xvelAvg - currentVx) * options.matchingFactor;
      nextVy = currentVy + (yposAvg - boidY) * options.centeringFactor +
               (yvelAvg - currentVy) * options.matchingFactor;
    }

    nextVx += closeDx * options.avoidFactor;
    nextVy += closeDy * options.avoidFactor;

    if (boidX < options.leftMargin) {
      nextVx += options.turnFactor;
    }
    if (boidX > options.rightMargin) {
      nextVx -= options.turnFactor;
    }
    if (boidY > options.bottomMargin) {
      nextVy -= options.turnFactor;
    }
    if (boidY < options.topMargin) {
      nextVy += options.turnFactor;
    }

    normalizeSpeed(options, nextVx, nextVy);

    boids.vx[i] = nextVx;
    boids.vy[i] = nextVy;
    boids.x[i] = boidX + nextVx;
    boids.y[i] = boidY + nextVy;
  }
}
