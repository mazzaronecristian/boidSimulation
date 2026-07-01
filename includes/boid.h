#pragma once
#include <cmath>
#include <cstdio>
#include <omp.h> // for OpenMP library functions
#include <vector>

class Grid;

class Boid {
public:
  int id;
  double x;
  double y;
  double vx;
  double vy;
  Boid *prev_;
  Boid *next_;

  Grid *grid = nullptr;
  Boid(Grid *grid, double x, double y);

  void move(double x, double y);
};

struct BoidSimulationDataList {
  std::vector<Boid> boids;
  int capacity = 0;
  int numBoids = 0;

  // params
  // TODO: rendi questi parametri configurabili
  float turnFactor = 0.2f;
  float visualRange = 150.0f;
  float protectedRange = 8.0f;
  float centeringFactor = 0.0005f;
  float avoidFactor = 0.05f;
  float matchingFactor = 0.05f;
  int maxSpeed = 6;
  int minSpeed = 3;
  float maxBias = 0.01f;
  float biasIncrement = 0.00004f;

  int topMargin = 0;
  int rightMargin = 0;
  int bottomMargin = 0;
  int leftMargin = 0;

  BoidSimulationDataList() = default;

  float squaredDistance(const Boid &first, const Boid &second) const {
    float dx = first.x - second.x;
    float dy = first.y - second.y;
    return dx * dx + dy * dy;
  }

  void allocate(int capacityToAllocate) {
    boids.clear();
    boids.reserve(capacityToAllocate);
    capacity = capacityToAllocate;
    numBoids = 0;
  }

  void addBoid(Boid boid) {
    if (numBoids == capacity)
      return;
    boids.push_back(boid);
    numBoids++;
  }

  int size() const { return numBoids; }

  void updateBoids() {
    for (int i = 0; i < size(); i++) {
      float xpos_avg = 0;
      float ypos_avg = 0;
      float xvel_avg = 0;
      float yvel_avg = 0;
      float closeDx = 0;
      float closeDy = 0;
      int neighboring_boids = 0;
      Boid &boid = boids[i];

      findNeighborsAndCalculateAvgs(boid, i, xpos_avg, ypos_avg, xvel_avg,
                                    yvel_avg, neighboring_boids, closeDx,
                                    closeDy);

      if (neighboring_boids > 0) {
        xpos_avg = xpos_avg / neighboring_boids;
        ypos_avg = ypos_avg / neighboring_boids;
        xvel_avg = xvel_avg / neighboring_boids;
        yvel_avg = yvel_avg / neighboring_boids;

        // Alignment and Cohesion
        boid.vx = (boid.vx + (xpos_avg - boid.x) * centeringFactor +
                   (xvel_avg - boid.vx) * matchingFactor);

        boid.vy = (boid.vy + (ypos_avg - boid.y) * centeringFactor +
                   (yvel_avg - boid.vy) * matchingFactor);
      }
      // Separation
      boid.vx = boid.vx + (closeDx * avoidFactor);
      boid.vy = boid.vy + (closeDy * avoidFactor);
      checkScreenEdges(boid);

      // TODO: bias

      normalizeSpeed(boid);

      // update position
      boid.x = boid.x + boid.vx;
      boid.y = boid.y + boid.vy;
    }
  }

  void findNeighborsAndCalculateAvgs(const Boid &boid, const int i,
                                     float &xpos_avg, float &ypos_avg,
                                     float &xvel_avg, float &yvel_avg,
                                     int &neighboring_boids, float &closeDx,
                                     float &closeDy) {

    const float visualRangeSquared = visualRange * visualRange;
    const float protectedRangeSquared = protectedRange * protectedRange;

    for (int j = 0; j < size(); j++) {
      if (i == j)
        continue;
      const Boid &otherBoid = boids[j];
      const float distanceSquared = squaredDistance(boid, otherBoid);

      if (distanceSquared < visualRangeSquared) {
        xvel_avg += otherBoid.vx;
        yvel_avg += otherBoid.vy;
        xpos_avg += otherBoid.x;
        ypos_avg += otherBoid.y;
        neighboring_boids += 1;
      }
      if (distanceSquared < protectedRangeSquared) {
        closeDx += boid.x - otherBoid.x;
        closeDy += boid.y - otherBoid.y;
      }
    }
  }

  void checkScreenEdges(Boid &boid) {
    if (boid.x < leftMargin)
      boid.vx = boid.vx + turnFactor;
    if (boid.x > rightMargin)
      boid.vx = boid.vx - turnFactor;
    if (boid.y > bottomMargin)
      boid.vy = boid.vy - turnFactor;
    if (boid.y < topMargin)
      boid.vy = boid.vy + turnFactor;
  }

  void normalizeSpeed(Boid &boid) {
    float speed = std::sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
    if (speed == 0.0f) {
      boid.vx = static_cast<float>(minSpeed);
      boid.vy = 0.0f;
      return;
    }

    // Enforce min and max speeds
    if (speed < minSpeed) {
      boid.vx = (boid.vx / speed) * minSpeed;
      boid.vy = (boid.vy / speed) * minSpeed;
    }
    if (speed > maxSpeed) {
      boid.vx = (boid.vx / speed) * maxSpeed;
      boid.vy = (boid.vy / speed) * maxSpeed;
    }
  }
};
