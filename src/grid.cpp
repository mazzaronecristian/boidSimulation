#include "grid.h"

void Grid::buildGrid(const BoidSoA &boids) {

  cells.clear();
  locations.clear();

  for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
    CellKey cellKey = cellFrom(boids.x[i], boids.y[i]);
    auto &cellBoids = cells[cellKey];
    cellBoids.push_back(i);
    locations[boids.id[i]] = {cellKey, cellBoids.size() - 1};
  }
}

void Grid::findNeighbors(const BoidSoA &boids, int i, float &xpos_avg,
                         float &ypos_avg, float &xvel_avg, float &yvel_avg,
                         int &neighboring_boids, float &closeDx,
                         float &closeDy) const {
  const double visibleRangeSquared = margin_.visualRange * margin_.visualRange;
  const float protectedRangeSquared =
      margin_.protectedRange * margin_.protectedRange;

  const float bx = boids.x[i];
  const float by = boids.y[i];

  const int minCx =
      static_cast<int>(
          std::floor((bx - margin_.visualRange) / margin_.visualRange));
  const int maxCx =
      static_cast<int>(
          std::floor((bx + margin_.visualRange) / margin_.visualRange));
  const int minCy =
      static_cast<int>(
          std::floor((by - margin_.visualRange) / margin_.visualRange));
  const int maxCy =
      static_cast<int>(
          std::floor((by + margin_.visualRange) / margin_.visualRange));

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

void Grid::move(BoidSoA &boids, int i) {
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

void Grid::applyRulesToBoid(BoidSoA &boids, int i, float xpos_avg,
                            float ypos_avg, float xvel_avg, float yvel_avg,
                            float closeDx, float closeDy,
                            int neighboring_boids) {
  if (neighboring_boids > 0) {
    xpos_avg /= neighboring_boids;
    ypos_avg /= neighboring_boids;
    xvel_avg /= neighboring_boids;
    yvel_avg /= neighboring_boids;

    boids.vx[i] = boids.vx[i] +
                  (xpos_avg - boids.x[i]) * margin_.centeringFactor +
                  (xvel_avg - boids.vx[i]) * margin_.matchingFactor;

    boids.vy[i] = boids.vy[i] +
                  (ypos_avg - boids.y[i]) * margin_.centeringFactor +
                  (yvel_avg - boids.vy[i]) * margin_.matchingFactor;
  }

  boids.vx[i] += closeDx * margin_.avoidFactor;
  boids.vy[i] += closeDy * margin_.avoidFactor;

  checkScreenEdges(boids, i);
  normalizeSpeed(boids, i);
}
