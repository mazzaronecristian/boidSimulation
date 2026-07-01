#pragma once
#include "boid2.h"
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

class Grid2 {
private:
  std::unordered_map<CellKey, std::vector<Boid2 *>, CellKeyHash> cells;
  std::unordered_map<int, std::pair<CellKey, std::size_t>> locations;

public:
  double visualRange = 40.0;
  float turnFactor = .3f;
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

  void buildGrid(std::vector<Boid2> &boids) {
    cells.clear();
    locations.clear();
    for (Boid2 &boid : boids) {
      CellKey cellKey = cellFrom(boid.x, boid.y);
      auto &cellBoids = cells[cellKey];
      cellBoids.push_back(&boid);
      locations[boid.id] = {cellKey, cellBoids.size() - 1};
    }
  }

  void findNeighbors(const Boid2 &boid, float &xpos_avg, float &ypos_avg,
                     float &xvel_avg, float &yvel_avg, int &neighboring_boids,
                     float &closeDx, float &closeDy) const {

    const double visibleRangeSquared = visualRange * visualRange;
    const float protectedRangeSquared = protectedRange * protectedRange;

    const int minCx =
        static_cast<int>(std::floor((boid.x - visualRange) / visualRange));
    const int maxCx =
        static_cast<int>(std::floor((boid.x + visualRange) / visualRange));
    const int minCy =
        static_cast<int>(std::floor((boid.y - visualRange) / visualRange));
    const int maxCy =
        static_cast<int>(std::floor((boid.y + visualRange) / visualRange));

    for (int cx = minCx; cx <= maxCx; ++cx) {
      for (int cy = minCy; cy <= maxCy; ++cy) {
        const auto it = cells.find(CellKey{cx, cy});
        if (it == cells.end()) {
          continue;
        }

        const std::vector<Boid2 *> &cellBoids = it->second;
        for (Boid2 *neighbor : cellBoids) {
          if (neighbor == &boid) {
            continue;
          }
          const double dx = neighbor->x - boid.x;
          const double dy = neighbor->y - boid.y;
          if (dx * dx + dy * dy <= visibleRangeSquared) {
            xvel_avg += neighbor->vx;
            yvel_avg += neighbor->vy;
            xpos_avg += neighbor->x;
            ypos_avg += neighbor->y;
            neighboring_boids += 1;
          }
          if (dx * dx + dy * dy < protectedRangeSquared) {
            closeDx += boid.x - neighbor->x;
            closeDy += boid.y - neighbor->y;
          }
        }
      }
    }
  }

  void move(Boid2 &boid) {
    double x = boid.x + boid.vx;
    double y = boid.y + boid.vy;

    const CellKey newCellKey = cellFrom(x, y);
    boid.x = x;
    boid.y = y;

    auto locationIt = locations.find(boid.id);
    if (locationIt == locations.end()) {
      auto &newCell = cells[newCellKey];
      newCell.push_back(&boid);
      locations[boid.id] = {newCellKey, newCell.size() - 1};
      return;
    }

    const CellKey oldCellKey = locationIt->second.first;
    const std::size_t oldIndex = locationIt->second.second;
    if (oldCellKey == newCellKey) {
      return;
    }

    auto oldCellIt = cells.find(oldCellKey);
    if (oldCellIt != cells.end() && oldIndex < oldCellIt->second.size()) {
      std::vector<Boid2 *> &oldCell = oldCellIt->second;
      Boid2 *movedBoid = oldCell[oldIndex];
      Boid2 *lastBoid = oldCell.back();
      oldCell[oldIndex] = lastBoid;
      oldCell.pop_back();

      if (lastBoid != movedBoid) {
        auto lastLocationIt = locations.find(lastBoid->id);
        if (lastLocationIt != locations.end()) {
          lastLocationIt->second.second = oldIndex;
        }
      }

      auto &newCell = cells[newCellKey];
      newCell.push_back(&boid);
      locations[boid.id] = {newCellKey, newCell.size() - 1};
      if (oldCell.empty()) {
        cells.erase(oldCellIt);
      }
      return;
    }

    auto &newCell = cells[newCellKey];
    newCell.push_back(&boid);
    locations[boid.id] = {newCellKey, newCell.size() - 1};
  }

  void applyRulesToBoid(Boid2 &boid, float xpos_avg, float ypos_avg,
                        float xvel_avg, float yvel_avg, float closeDx,
                        float closeDy, int neighboring_boids

  ) {
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
    normalizeSpeed(boid);
  }

  void checkScreenEdges(Boid2 &boid) {
    if (boid.x < leftMargin)
      boid.vx = boid.vx + turnFactor;
    if (boid.x > rightMargin)
      boid.vx = boid.vx - turnFactor;
    if (boid.y > bottomMargin)
      boid.vy = boid.vy - turnFactor;
    if (boid.y < topMargin)
      boid.vy = boid.vy + turnFactor;
  }

  void normalizeSpeed(Boid2 &boid) {
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
