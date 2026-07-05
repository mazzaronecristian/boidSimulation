#pragma once
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

struct BoidSoA {
  std::vector<int> id;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> vx;
  std::vector<float> vy;

  std::size_t size() const { return id.size(); }

  void push_back(int i, float px, float py, float pvx, float pvy) {
    id.push_back(i);
    x.push_back(px);
    y.push_back(py);
    vx.push_back(pvx);
    vy.push_back(pvy);
  }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey &key) const noexcept {
    const std::size_t h1 = std::hash<int32_t>{}(key.cx);
    const std::size_t h2 = std::hash<int32_t>{}(key.cy);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

class Grid2 {
private:
  std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells;
  std::unordered_map<int, std::pair<CellKey, std::size_t>> locations;

public:
  double visualRange = 40.0;
  float turnFactor = .2f;
  float protectedRange = 10;
  float centeringFactor = 0.003f;
  float avoidFactor = 0.15f;
  float matchingFactor = 0.1f;
  int maxSpeed = 6;
  int minSpeed = 3;

  int topMargin = 0;
  int rightMargin = 0;
  int bottomMargin = 0;
  int leftMargin = 0;

  int size() { return locations.size(); }
  int getMaxSpeed() { return maxSpeed; }

  int getMinSpeed() { return minSpeed; }

  explicit Grid2(int topMargin, int rightMargin, int bottomMargin,
                 int leftMargin)
      : topMargin(topMargin), rightMargin(rightMargin),
        bottomMargin(bottomMargin), leftMargin(leftMargin) {}

  CellKey cellFrom(double x, double y) const {
    return CellKey{static_cast<int32_t>(std::floor(x / visualRange)),
                   static_cast<int32_t>(std::floor(y / visualRange))};
  }

  void buildGrid(const BoidSoA &boids) {
    cells.clear();
    locations.clear();

    for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
      CellKey cellKey = cellFrom(boids.x[i], boids.y[i]);
      auto &cellBoids = cells[cellKey];
      cellBoids.push_back(i);
      locations[boids.id[i]] = {cellKey, cellBoids.size() - 1};
    }
  }

  void findNeighbors(const BoidSoA &boids, int i, float &xpos_avg,
                     float &ypos_avg, float &xvel_avg, float &yvel_avg,
                     int &neighboring_boids, float &closeDx,
                     float &closeDy) const {
    const double visibleRangeSquared = visualRange * visualRange;
    const float protectedRangeSquared = protectedRange * protectedRange;

    const float bx = boids.x[i];
    const float by = boids.y[i];

    const int minCx =
        static_cast<int>(std::floor((bx - visualRange) / visualRange));
    const int maxCx =
        static_cast<int>(std::floor((bx + visualRange) / visualRange));
    const int minCy =
        static_cast<int>(std::floor((by - visualRange) / visualRange));
    const int maxCy =
        static_cast<int>(std::floor((by + visualRange) / visualRange));

    for (int cx = minCx; cx <= maxCx; ++cx) {
      for (int cy = minCy; cy <= maxCy; ++cy) {
        const auto it = cells.find(CellKey{cx, cy});
        if (it == cells.end())
          continue;

        const std::vector<int> &cellBoids = it->second;
        for (int j : cellBoids) {
          if (j == i)
            continue;

          const double dx = boids.x[j] - bx;
          const double dy = boids.y[j] - by;
          const double dist2 = dx * dx + dy * dy;

          if (dist2 <= visibleRangeSquared) {
            xvel_avg += boids.vx[j];
            yvel_avg += boids.vy[j];
            xpos_avg += boids.x[j];
            ypos_avg += boids.y[j];
            neighboring_boids += 1;
          }

          if (dist2 < protectedRangeSquared) {
            closeDx += bx - boids.x[j];
            closeDy += by - boids.y[j];
          }
        }
      }
    }
  }

  void move(BoidSoA &boids, int i) {
    boids.x[i] += boids.vx[i];
    boids.y[i] += boids.vy[i];

    const CellKey newCellKey = cellFrom(boids.x[i], boids.y[i]);

    auto locationIt = locations.find(boids.id[i]);
    if (locationIt == locations.end()) {
      auto &newCell = cells[newCellKey];
      newCell.push_back(i);
      locations[boids.id[i]] = {newCellKey, newCell.size() - 1};
      return;
    }

    const CellKey oldCellKey = locationIt->second.first;
    const std::size_t oldIndex = locationIt->second.second;
    if (oldCellKey == newCellKey)
      return;

    auto oldCellIt = cells.find(oldCellKey);
    if (oldCellIt != cells.end() && oldIndex < oldCellIt->second.size()) {
      auto &oldCell = oldCellIt->second;
      int movedIndex = oldCell[oldIndex];
      int lastIndex = oldCell.back();
      oldCell[oldIndex] = lastIndex;
      oldCell.pop_back();

      if (lastIndex != movedIndex) {
        auto lastLocationIt = locations.find(boids.id[lastIndex]);
        if (lastLocationIt != locations.end()) {
          lastLocationIt->second.second = oldIndex;
        }
      }

      auto &newCell = cells[newCellKey];
      newCell.push_back(i);
      locations[boids.id[i]] = {newCellKey, newCell.size() - 1};

      if (oldCell.empty()) {
        cells.erase(oldCellIt);
      }
      return;
    }

    auto &newCell = cells[newCellKey];
    newCell.push_back(i);
    locations[boids.id[i]] = {newCellKey, newCell.size() - 1};
  }

  void applyRulesToBoid(BoidSoA &boids, int i, float xpos_avg, float ypos_avg,
                        float xvel_avg, float yvel_avg, float closeDx,
                        float closeDy, int neighboring_boids) {
    if (neighboring_boids > 0) {
      xpos_avg /= neighboring_boids;
      ypos_avg /= neighboring_boids;
      xvel_avg /= neighboring_boids;
      yvel_avg /= neighboring_boids;

      boids.vx[i] = boids.vx[i] + (xpos_avg - boids.x[i]) * centeringFactor +
                    (xvel_avg - boids.vx[i]) * matchingFactor;

      boids.vy[i] = boids.vy[i] + (ypos_avg - boids.y[i]) * centeringFactor +
                    (yvel_avg - boids.vy[i]) * matchingFactor;
    }

    boids.vx[i] += closeDx * avoidFactor;
    boids.vy[i] += closeDy * avoidFactor;

    checkScreenEdges(boids, i);
    normalizeSpeed(boids, i);
  }

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
};
