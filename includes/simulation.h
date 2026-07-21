#pragma once

#include "boid_soa.h"
#include "grid.h"

void initSimulation(Grid &grid, BoidSoA &boids, int size);
void runParallelSoA(Grid &grid, BoidSoA &boids);
