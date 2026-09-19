#pragma once

#include "boid_soa.h"
#include "grid.h"

void initParallelSimulation(Grid &grid, BoidSoA &boids, int size);
void placeBoids(const Options &margin, BoidSoA &boids, int size);
void runParallelSoA(Grid &grid, BoidSoA &boids);
void runSequential(const Options &margin, BoidSoA &boids);
