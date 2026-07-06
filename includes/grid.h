#pragma once
#include "boid_soa.h"
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

typedef struct CellKey {
  int32_t cx, cy;
  bool operator==(const CellKey &o) const { return cx == o.cx && cy == o.cy; }
} CellKey;

struct CellKeyHash {
  std::size_t operator()(const CellKey &key) const noexcept {
    const std::size_t h1 = std::hash<int32_t>{}(key.cx);
    const std::size_t h2 = std::hash<int32_t>{}(key.cy);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

class Grid {
private:
  double visualRange = 40.0;
  float turnFactor = .2f;
  float protectedRange = 10;
  float centeringFactor = 0.003f;
  float avoidFactor = 0.15f;
  float matchingFactor = 0.1f;
  int maxSpeed = 6;
  int minSpeed = 3;

  std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells;
  std::unordered_map<int, std::pair<CellKey, std::size_t>> locations;

  void checkScreenEdges(BoidSoA &boids, int i) {
    if (boids.x[i] < leftMargin)
      boids.vx[i] += turnFactor;
    if (boids.x[i] > rightMargin)
      boids.vx[i] -= turnFactor;
    if (boids.y[i] > bottomMargin)
      boids.vy[i] -= turnFactor;
    if (boids.y[i] < topMargin)
      boids.vy[i] += turnFactor;
  }

  void normalizeSpeed(BoidSoA &boids, int i) {
    float speed =
        std::sqrt(boids.vx[i] * boids.vx[i] + boids.vy[i] * boids.vy[i]);
    if (speed == 0.0f) {
      boids.vx[i] = static_cast<float>(minSpeed);
      boids.vy[i] = 0.0f;
      return;
    }

    if (speed < minSpeed) {
      boids.vx[i] = (boids.vx[i] / speed) * minSpeed;
      boids.vy[i] = (boids.vy[i] / speed) * minSpeed;
    }
    if (speed > maxSpeed) {
      boids.vx[i] = (boids.vx[i] / speed) * maxSpeed;
      boids.vy[i] = (boids.vy[i] / speed) * maxSpeed;
    }
  }

public:
  int topMargin = 0;
  int rightMargin = 0;
  int bottomMargin = 0;
  int leftMargin = 0;
  int size() const { return static_cast<int>(locations.size()); }

  double getVisualRange() const { return visualRange; }
  void setVisualRange(double value) { visualRange = value; }

  float getTurnFactor() const { return turnFactor; }
  void setTurnFactor(float value) { turnFactor = value; }

  float getProtectedRange() const { return protectedRange; }
  void setProtectedRange(float value) { protectedRange = value; }

  float getCenteringFactor() const { return centeringFactor; }
  void setCenteringFactor(float value) { centeringFactor = value; }

  float getAvoidFactor() const { return avoidFactor; }
  void setAvoidFactor(float value) { avoidFactor = value; }

  float getMatchingFactor() const { return matchingFactor; }
  void setMatchingFactor(float value) { matchingFactor = value; }

  int getMaxSpeed() const { return maxSpeed; }
  void setMaxSpeed(int value) { maxSpeed = value; }

  int getMinSpeed() const { return minSpeed; }
  void setMinSpeed(int value) { minSpeed = value; }

  explicit Grid(int topMargin, int rightMargin, int bottomMargin,
                int leftMargin)
      : topMargin(topMargin), rightMargin(rightMargin),
        bottomMargin(bottomMargin), leftMargin(leftMargin) {}

  CellKey cellFrom(double x, double y) const {
    return CellKey{static_cast<int32_t>(std::floor(x / visualRange)),
                   static_cast<int32_t>(std::floor(y / visualRange))};
  }

  void buildGrid(const BoidSoA &boids);

  void findNeighbors(const BoidSoA &boids, int i, float &xpos_avg,
                     float &ypos_avg, float &xvel_avg, float &yvel_avg,
                     int &neighboring_boids, float &closeDx,
                     float &closeDy) const;

  void move(BoidSoA &boids, int i);

  void applyRulesToBoid(BoidSoA &boids, int i, float xpos_avg, float ypos_avg,
                        float xvel_avg, float yvel_avg, float closeDx,
                        float closeDy, int neighboring_boids);
};
