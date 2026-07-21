#include "simulation.h"

#include <cstdlib>
#include <omp.h>

void initSimulation(Grid &grid, BoidSoA &boids, int size) {
  boids.init(size);
  for (int i = 0; i < size; ++i) {
    float x = static_cast<float>(
        rand() % (grid.rightMargin - grid.leftMargin + 1) + grid.leftMargin);
    float y = static_cast<float>(
        rand() % (grid.bottomMargin - grid.topMargin + 1) + grid.topMargin);
    float vx = static_cast<float>((rand() % (grid.getMaxSpeed() * 2 + 1)) -
                                  grid.getMaxSpeed());
    float vy = static_cast<float>((rand() % (grid.getMaxSpeed() * 2 + 1)) -
                                  grid.getMaxSpeed());
    boids.push_back(i, x, y, vx, vy);
  }
  grid.buildGrid(boids);
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
}
