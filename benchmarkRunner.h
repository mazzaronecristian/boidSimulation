#pragma once

#include "grid.h"
#include "simulation.h"

#include <memory>
#include <stdexcept>
#include <string_view>

namespace benchmark {

inline constexpr int kScreenWidth = 1920;
inline constexpr int kScreenHeight = 1080;
inline constexpr int kMargin = 300;

inline Grid createBenchmarkGrid() {
  return Grid(kMargin, kScreenWidth - kMargin, kScreenHeight - kMargin,
              kMargin);
}

inline Margin createBenchmarkMargin() {
  Margin margin;
  margin.topMargin = kMargin;
  margin.rightMargin = kScreenWidth - kMargin;
  margin.bottomMargin = kScreenHeight - kMargin;
  margin.leftMargin = kMargin;
  return margin;
}

class BenchmarkRunner {
public:
  explicit BenchmarkRunner(int boidCount) : boidCount_(boidCount) {}
  virtual ~BenchmarkRunner() = default;

  virtual const char *name() const = 0;
  virtual void init() = 0;
  virtual void simulateStep() = 0;

protected:
  int boidCount() const { return boidCount_; }

private:
  int boidCount_;
};

class ParallelBenchmarkRunner final : public BenchmarkRunner {
public:
  explicit ParallelBenchmarkRunner(int boidCount)
      : BenchmarkRunner(boidCount), grid_(createBenchmarkGrid()) {}

  const char *name() const override { return "parallel"; }

  void init() override {
    grid_ = createBenchmarkGrid();
    boids_ = BoidSoA{};
    initParallelSimulation(grid_, boids_, boidCount());
  }

  void simulateStep() override { runParallelSoA(grid_, boids_); }

private:
  Grid grid_;
  BoidSoA boids_;
};

class SequentialBenchmarkRunner final : public BenchmarkRunner {
public:
  explicit SequentialBenchmarkRunner(int boidCount)
      : BenchmarkRunner(boidCount), margin_(createBenchmarkMargin()) {}

  const char *name() const override { return "sequential"; }

  void init() override {
    boids_ = BoidSoA{};
    placeBoids(margin_, boids_, boidCount());
  }

  void simulateStep() override { runSequential(margin_, boids_); }

private:
  Margin margin_;
  BoidSoA boids_;
};

inline std::unique_ptr<BenchmarkRunner>
createBenchmarkRunner(std::string_view mode, int boidCount) {
  if (mode == "parallel") {
    return std::make_unique<ParallelBenchmarkRunner>(boidCount);
  }
  if (mode == "sequential") {
    return std::make_unique<SequentialBenchmarkRunner>(boidCount);
  }
  throw std::invalid_argument(
      "Usage: benchmark <parallel|sequential> <n_iterations>");
}

} // namespace benchmark
