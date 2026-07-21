#pragma once
#include "boid_soa.h"
#include "margin.h"
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
  Margin margin_;

  std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells;
  std::unordered_map<int, std::pair<CellKey, std::size_t>> locations;

  void checkScreenEdges(BoidSoA &boids, int i) {
    if (boids.x[i] < margin_.leftMargin)
      boids.vx[i] += margin_.turnFactor;
    if (boids.x[i] > margin_.rightMargin)
      boids.vx[i] -= margin_.turnFactor;
    if (boids.y[i] > margin_.bottomMargin)
      boids.vy[i] -= margin_.turnFactor;
    if (boids.y[i] < margin_.topMargin)
      boids.vy[i] += margin_.turnFactor;
  }

  void normalizeSpeed(BoidSoA &boids, int i) {
    float speed =
        std::sqrt(boids.vx[i] * boids.vx[i] + boids.vy[i] * boids.vy[i]);
    if (speed == 0.0f) {
      boids.vx[i] = static_cast<float>(margin_.minSpeed);
      boids.vy[i] = 0.0f;
      return;
    }

    if (speed < margin_.minSpeed) {
      boids.vx[i] = (boids.vx[i] / speed) * margin_.minSpeed;
      boids.vy[i] = (boids.vy[i] / speed) * margin_.minSpeed;
    }
    if (speed > margin_.maxSpeed) {
      boids.vx[i] = (boids.vx[i] / speed) * margin_.maxSpeed;
      boids.vy[i] = (boids.vy[i] / speed) * margin_.maxSpeed;
    }
  }

public:
  int size() const { return static_cast<int>(locations.size()); }

  const Margin &getMargin() const { return margin_; }

  double getVisualRange() const { return margin_.visualRange; }
  void setVisualRange(double value) { margin_.visualRange = value; }

  float getTurnFactor() const { return margin_.turnFactor; }
  void setTurnFactor(float value) { margin_.turnFactor = value; }

  float getProtectedRange() const { return margin_.protectedRange; }
  void setProtectedRange(float value) { margin_.protectedRange = value; }

  float getCenteringFactor() const { return margin_.centeringFactor; }
  void setCenteringFactor(float value) { margin_.centeringFactor = value; }

  float getAvoidFactor() const { return margin_.avoidFactor; }
  void setAvoidFactor(float value) { margin_.avoidFactor = value; }

  float getMatchingFactor() const { return margin_.matchingFactor; }
  void setMatchingFactor(float value) { margin_.matchingFactor = value; }

  int getMaxSpeed() const { return margin_.maxSpeed; }
  void setMaxSpeed(int value) { margin_.maxSpeed = value; }

  int getMinSpeed() const { return margin_.minSpeed; }
  void setMinSpeed(int value) { margin_.minSpeed = value; }

  explicit Grid(const Margin &margin) : margin_(margin) {}

  Grid(int topMargin, int rightMargin, int bottomMargin, int leftMargin)
      : margin_{} {
    margin_.topMargin = topMargin;
    margin_.rightMargin = rightMargin;
    margin_.bottomMargin = bottomMargin;
    margin_.leftMargin = leftMargin;
  }

  CellKey cellFrom(double x, double y) const {
    return CellKey{
        static_cast<int32_t>(std::floor(x / margin_.visualRange)),
        static_cast<int32_t>(std::floor(y / margin_.visualRange))};
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
