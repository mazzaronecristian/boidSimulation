#include "grid.h"
#include "simulation.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using SimulationInit = void (*)(Grid &, BoidSoA &, int);
using SimulationStep = void (*)(Grid &, BoidSoA &);

struct BenchmarkConfiguration {
  const char *name;
  SimulationInit init;
  SimulationStep step;
  bool implemented;
};

constexpr std::array<int, 5> kBoidCounts = {10, 100, 1000, 10000, 100000};
constexpr int kScreenWidth = 1920;
constexpr int kScreenHeight = 1080;
constexpr int kMargin = 300;

Grid createBenchmarkGrid() {
  return Grid(kMargin, kScreenWidth - kMargin, kScreenHeight - kMargin,
              kMargin);
}

const BenchmarkConfiguration &getConfiguration(std::string_view mode) {
  static const BenchmarkConfiguration parallelConfiguration = {
      "parallel", initSimulation, runParallelSoA, true};
  static const BenchmarkConfiguration sequentialConfiguration = {
      "sequential", nullptr, nullptr, false};

  if (mode == "parallel") {
    return parallelConfiguration;
  }
  if (mode == "sequential") {
    return sequentialConfiguration;
  }

  throw std::invalid_argument(
      "Usage: benchmark <parallel|sequential> <n_iterations>");
}

int parseIterations(const char *value) {
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed <= 0) {
    throw std::invalid_argument("n_iterations must be a positive integer.");
  }
  return static_cast<int>(parsed);
}

std::string buildOutputFileName(const BenchmarkConfiguration &configuration,
                                int iterations) {
  std::ostringstream fileName;
  fileName << "benchmark_" << configuration.name << '_' << iterations
           << ".csv";
  return fileName.str();
}

void runBenchmarks(const BenchmarkConfiguration &configuration,
                   int iterations) {
  std::ofstream csvFile(buildOutputFileName(configuration, iterations));
  if (!csvFile.is_open()) {
    throw std::runtime_error("Unable to open CSV output file.");
  }

  auto writeLine = [&](const std::string &line) {
    std::cout << line << '\n';
    csvFile << line << '\n';
  };

  writeLine("mode: " + std::string(configuration.name));
  writeLine("iterations: " + std::to_string(iterations));
  writeLine("boids,total_ms,avg_ms");

  for (int boidCount : kBoidCounts) {
    std::srand(0);

    Grid grid = createBenchmarkGrid();
    BoidSoA boids;
    configuration.init(grid, boids, boidCount);

    const auto start = std::chrono::steady_clock::now();
    for (int iteration = 0; iteration < iterations; ++iteration) {
      configuration.step(grid, boids);
    }
    const auto end = std::chrono::steady_clock::now();

    const double totalMs =
        std::chrono::duration<double, std::milli>(end - start).count();
    const double averageMs = totalMs / static_cast<double>(iterations);

    std::ostringstream line;
    line << std::fixed << std::setprecision(3) << boidCount << ',' << totalMs
         << ',' << averageMs;
    writeLine(line.str());
  }
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: benchmark <parallel|sequential> <n_iterations>\n";
    return EXIT_FAILURE;
  }

  try {
    const BenchmarkConfiguration &configuration = getConfiguration(argv[1]);
    const int iterations = parseIterations(argv[2]);
    if (!configuration.implemented) {
      throw std::logic_error("Sequential benchmark not implemented yet.");
    }
    runBenchmarks(configuration, iterations);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
