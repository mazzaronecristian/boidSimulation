#pragma once

#include "boid.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

// Fixed-grid spatial partition based on the pattern described in
// https://gameprogrammingpatterns.com/spatial-partition.html
//
// Objects are stored in grid cells based on their position. Each cell keeps a
// doubly linked list of objects, so moving an object across cell boundaries is
// cheap: unlink from the old cell and insert into the new one.
class SpatialPartition {
public:
  struct CellLocation {
    int x = 0;
    int y = 0;
  };

  SpatialPartition(int worldWidth, int worldHeight, int cellSize)
      : worldWidth_(std::max(0, worldWidth)),
        worldHeight_(std::max(0, worldHeight)),
        cellSize_(std::max(1, cellSize)),
        cols_(std::max(1, (worldWidth_ + cellSize_ - 1) / cellSize_)),
        rows_(std::max(1, (worldHeight_ + cellSize_ - 1) / cellSize_)),
        cells_(static_cast<std::size_t>(cols_ * rows_)) {}

  SpatialPartition(int worldWidth, int worldHeight)
      : SpatialPartition(worldWidth, worldHeight, 20) {}

  void clear() {
    for (Cell &cell : cells_) {
      cell.head = nullptr;
    }
    nodes_.clear();
  }

  CellLocation cellFor(double x, double y) const {
    return {clampCellX(static_cast<int>(x / cellSize_)),
            clampCellY(static_cast<int>(y / cellSize_))};
  }

  void add(Boid *boid) {
    if (boid == nullptr) {
      return;
    }

    auto node = std::make_unique<Node>();
    node->boid = boid;
    node->cell = cellFor(boid->x, boid->y);
    insertFront(*node);
    nodes_[boid] = std::move(node);
  }

  void remove(Boid *boid) {
    auto it = nodes_.find(boid);
    if (it == nodes_.end()) {
      return;
    }

    unlink(*it->second);
    nodes_.erase(it);
  }

  void move(Boid *boid, double x, double y) {
    if (boid == nullptr) {
      return;
    }

    auto it = nodes_.find(boid);
    if (it == nodes_.end()) {
      boid->x = x;
      boid->y = y;
      add(boid);
      return;
    }

    Node &node = *it->second;
    CellLocation newCell = cellFor(x, y);

    boid->x = x;
    boid->y = y;

    if (node.cell.x == newCell.x && node.cell.y == newCell.y) {
      return;
    }

    unlink(node);
    node.cell = newCell;
    insertFront(node);
  }

  template <typename Fn>
  void forEachInCell(int cellX, int cellY, Fn &&fn) const {
    if (!isInside(cellX, cellY)) {
      return;
    }

    const Cell &cell = cells_[index(cellX, cellY)];
    for (Node *node = cell.head; node != nullptr; node = node->next) {
      fn(*node->boid);
    }
  }

  template <typename Fn>
  void forEachNearby(double x, double y, int radiusCells, Fn &&fn) const {
    CellLocation center = cellFor(x, y);
    const int minX = std::max(0, center.x - radiusCells);
    const int maxX = std::min(cols_ - 1, center.x + radiusCells);
    const int minY = std::max(0, center.y - radiusCells);
    const int maxY = std::min(rows_ - 1, center.y + radiusCells);

    for (int cellY = minY; cellY <= maxY; ++cellY) {
      for (int cellX = minX; cellX <= maxX; ++cellX) {
        forEachInCell(cellX, cellY, fn);
      }
    }
  }

  int columns() const { return cols_; }
  int rows() const { return rows_; }
  int cellSize() const { return cellSize_; }

private:
  struct Node {
    Boid *boid = nullptr;
    CellLocation cell;
    Node *prev = nullptr;
    Node *next = nullptr;
  };

  struct Cell {
    Node *head = nullptr;
  };

  int worldWidth_ = 0;
  int worldHeight_ = 0;
  int cellSize_ = 20;
  int cols_ = 1;
  int rows_ = 1;
  std::vector<Cell> cells_;
  std::unordered_map<Boid *, std::unique_ptr<Node>> nodes_;

  bool isInside(int cellX, int cellY) const {
    return cellX >= 0 && cellX < cols_ && cellY >= 0 && cellY < rows_;
  }

  std::size_t index(int cellX, int cellY) const {
    return static_cast<std::size_t>(cellY * cols_ + cellX);
  }

  int clampCellX(int cellX) const { return std::clamp(cellX, 0, cols_ - 1); }

  int clampCellY(int cellY) const { return std::clamp(cellY, 0, rows_ - 1); }

  void insertFront(Node &node) {
    Cell &cell = cells_[index(node.cell.x, node.cell.y)];
    node.prev = nullptr;
    node.next = cell.head;
    if (cell.head != nullptr) {
      cell.head->prev = &node;
    }
    cell.head = &node;
  }

  void unlink(Node &node) {
    Cell &cell = cells_[index(node.cell.x, node.cell.y)];
    if (node.prev != nullptr) {
      node.prev->next = node.next;
    } else if (cell.head == &node) {
      cell.head = node.next;
    }

    if (node.next != nullptr) {
      node.next->prev = node.prev;
    }

    node.prev = nullptr;
    node.next = nullptr;
  }
};
