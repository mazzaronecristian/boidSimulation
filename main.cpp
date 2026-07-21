#include "grid.h"
#include "simulation.h"
#include <cstdio>
#include <cstdlib>
#include <raylib.h>
#include <stdio.h>

// void fillBoidSimulation(BoidSimulationDataList &boidSimulationDataList,
//                         int capacity, int minX, int maxX, int minY, int
//                         maxY);
//
//
void drawParameters(Grid &grid);
void drawBoids(Grid &grid, BoidSoA &boids);

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
  Grid grid = Grid(minY, maxX, maxY, minX);
  BoidSoA boids;
  initSimulation(grid, boids, size);

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
    drawParameters(grid);
    runParallelSoA(grid, boids);
    drawBoids(grid, boids);

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

void drawParameters(Grid &grid) {

  char paramsText[256];
  std::snprintf(paramsText, sizeof(paramsText),
                "visualRange: %.1f  turnFactor: %.2f  protectedRange: %.1f",
                grid.getVisualRange(), grid.getTurnFactor(),
                grid.getProtectedRange());
  DrawText(paramsText, 10, 35, 20, RAYWHITE);
  std::snprintf(
      paramsText, sizeof(paramsText),
      "centeringFactor: %.3f  avoidFactor: %.2f  matchingFactor: %.2f",
      grid.getCenteringFactor(), grid.getAvoidFactor(),
      grid.getMatchingFactor());
  DrawText(paramsText, 10, 60, 20, RAYWHITE);

  std::snprintf(paramsText, sizeof(paramsText), "maxSpeed: %d  minSpeed: %d",
                grid.getMaxSpeed(), grid.getMinSpeed());
  DrawText(paramsText, 10, 85, 20, RAYWHITE);
}

void drawBoids(Grid &grid, BoidSoA &boids) {
  for (int i = 0; i < static_cast<int>(boids.size()); ++i) {
    grid.move(boids, i);

    Vector2 apex = {boids.x[i], boids.y[i]};
    Vector2 baseLeft = {boids.x[i] - 3.5f, boids.y[i] + 10.0f};
    Vector2 baseRight = {boids.x[i] + 3.5f, boids.y[i] + 10.0f};
    DrawTriangle(apex, baseLeft, baseRight, SKYBLUE);
  }
}
