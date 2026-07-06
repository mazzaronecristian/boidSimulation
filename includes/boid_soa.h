#pragma once
#include <cmath>
#include <vector>

struct BoidSoA {
  std::vector<int> id;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> vx;
  std::vector<float> vy;

  std::size_t size() const { return id.size(); }

  void init(int size) {
    id.reserve(size);
    x.reserve(size);
    y.reserve(size);
    vx.reserve(size);
    vy.reserve(size);
  }

  void push_back(int i, float px, float py, float pvx, float pvy) {
    id.push_back(i);
    x.push_back(px);
    y.push_back(py);
    vx.push_back(pvx);
    vy.push_back(pvy);
  }
};
