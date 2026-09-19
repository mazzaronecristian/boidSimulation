#pragma once

#include "boid_soa.h"
#include "grid.h"
#include "scheduling_strategy_enum.h"

void initParallelSimulation(Grid &grid, BoidSoA &boids, int size);
void placeBoids(const Options &margin, BoidSoA &boids, int size);
void runParallelSoA(Grid &grid, BoidSoA &boids,
                    ScheudulingStrategyEnum schedulingStrategy);
void runSequential(const Options &margin, BoidSoA &boids);
