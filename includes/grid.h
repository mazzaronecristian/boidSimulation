#pragma once

class Boid;

class Grid {
public:
  Grid() {
    for (int x = 0; x < NUM_CELLS; x++) {
      for (int y = 0; y < NUM_CELLS; y++) {
        cells_[x][y] = nullptr;
      }
    }
  }
  void add(Boid *unit);
  void move(Boid *unit, double x, double y);
  void handleCell(Boid *unit);

  static const int NUM_CELLS = 10;
  static const int CELL_SIZE = 20;

private:
  Boid *cells_[NUM_CELLS][NUM_CELLS];
};
