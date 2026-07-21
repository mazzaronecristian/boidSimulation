#pragma once

struct Margin {
  int topMargin = 0;
  int rightMargin = 0;
  int bottomMargin = 0;
  int leftMargin = 0;

  double visualRange = 40.0;
  float turnFactor = .2f;
  float protectedRange = 10;
  float centeringFactor = 0.003f;
  float avoidFactor = 0.15f;
  float matchingFactor = 0.1f;
  int maxSpeed = 6;
  int minSpeed = 3;
};
