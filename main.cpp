#include "boid2.h"
#include "grid2.h"
#include <cstdio>
#include <cstdlib>
#include <omp.h> // for OpenMP library functions
#include <raylib.h>
#include <stdio.h>

// void fillBoidSimulation(BoidSimulationDataList &boidSimulationDataList,
//                         int capacity, int minX, int maxX, int minY, int
//                         maxY);

int main(int argc, char **argv) {
  int size = 10000; // valore default

  if (argc > 1) {
    size = std::atoi(argv[1]);
  }
  // Initialization
  //--------------------------------------------------------------------------------------
  InitWindow(0, 0, "Boid Simulation");
  ToggleFullscreen();
  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
  int monitor = GetCurrentMonitor();
  const int SCREEN_WIDTH = GetMonitorWidth(monitor);
  const int SCREEN_HEIGHT = GetMonitorHeight(monitor);
  int margin = 300;
  int minX = margin;
  int maxX = GetScreenWidth() - margin;
  int minY = margin;
  int maxY = GetScreenHeight() - margin;
  printf("SCREEN_WIDTH %d\n", SCREEN_WIDTH);
  printf("SCREEN_HEIGHT %d\n", SCREEN_HEIGHT);
  //--------------------------------------------------------------------------------------

  Grid2 grid = Grid2(minY, maxX, maxY, minX);

  std::vector<Boid2> boids;
  for (int i = 0; i < size; i++) {
    float x = static_cast<float>(rand() % (maxX - minX + 1) + minX);
    float y = static_cast<float>(rand() % (maxY - minY + 1) + minY);
    float vx = static_cast<float>((rand() % (grid.getMaxSpeed() * 2 + 1)) -
                                  grid.getMaxSpeed());
    float vy = static_cast<float>((rand() % (grid.getMaxSpeed() * 2 + 1)) -
                                  grid.getMaxSpeed());

    Boid2 boid = {i, x, y, vx, vy};
    boids.push_back(boid);
  }
  grid.buildGrid(boids);

  printf("simulation size: %d\n", grid.size());
  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(BLACK);

    DrawRectangleLines(margin, margin, GetScreenWidth() - margin * 2,
                       GetScreenHeight() - margin * 2,
                       DARKGRAY); // NOTE: Uses QUADS internally, not lines
    DrawFPS(10, 10);

    char paramsText[256];
    std::snprintf(paramsText, sizeof(paramsText),
                  "visualRange: %.1f  turnFactor: %.2f  protectedRange: %.1f",
                  grid.visualRange, grid.turnFactor, grid.protectedRange);
    DrawText(paramsText, 10, 35, 20, RAYWHITE);

    std::snprintf(
        paramsText, sizeof(paramsText),
        "centeringFactor: %.3f  avoidFactor: %.2f  matchingFactor: %.2f",
        grid.centeringFactor, grid.avoidFactor, grid.matchingFactor);
    DrawText(paramsText, 10, 60, 20, RAYWHITE);

    std::snprintf(paramsText, sizeof(paramsText), "maxSpeed: %d  minSpeed: %d",
                  grid.maxSpeed, grid.minSpeed);
    DrawText(paramsText, 10, 85, 20, RAYWHITE);
    //
    const size_t n = boids.size();
    std::vector<float> xpos_avg(n, 0.0f);
    std::vector<float> ypos_avg(n, 0.0f);
    std::vector<float> xvel_avg(n, 0.0f);
    std::vector<float> yvel_avg(n, 0.0f);
    std::vector<float> closeDx(n, 0.0f);
    std::vector<float> closeDy(n, 0.0f);
    std::vector<int> neighboring_boids(n, 0);

#pragma omp parallel for default(none) num_threads(12)                         \
    shared(boids, grid, size, xpos_avg, ypos_avg, xvel_avg, yvel_avg, closeDx, \
               closeDy, neighboring_boids)
    for (Boid2 &boid : boids) {
      int i = boid.id;

      grid.findNeighbors(boid, xpos_avg[i], ypos_avg[i], xvel_avg[i],
                         yvel_avg[i], neighboring_boids[i], closeDx[i],
                         closeDy[i]);
      grid.applyRulesToBoid(boid, xpos_avg[i], ypos_avg[i], xvel_avg[i],
                            yvel_avg[i], closeDx[i], closeDy[i],
                            neighboring_boids[i]);
    }

    for (Boid2 &boid : boids) {
      grid.move(boid);

      const float halfBase = 3.5f;
      const float height = 10.0f;
      const float x = static_cast<float>(boid.x);
      const float y = static_cast<float>(boid.y);
      Vector2 apex = {x, y};
      Vector2 baseLeft = {x - halfBase, y + height};
      Vector2 baseRight = {x + halfBase, y + height};

      DrawTriangle(apex, baseLeft, baseRight, SKYBLUE);
    }

    EndDrawing();
  }

  //----------------------------------------------------------------------------------
  //

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------

  return 0;
}
