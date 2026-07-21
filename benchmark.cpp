#include "benchmarkRunner.h"

#include <array>
#include <chrono>
#include <cmath>
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

constexpr std::array<int, 5> kBoidCounts = {10, 100, 1000, 10000, 100000};

struct RunningStats {
  int count = 0;
  double total = 0.0;
  double mean = 0.0;
  double m2 = 0.0;

  void add(double sample) {
    ++count;
    total += sample;

    const double delta = sample - mean;
    mean += delta / static_cast<double>(count);
    const double delta2 = sample - mean;
    m2 += delta * delta2;
  }

  double standardDeviation() const {
    if (count <= 1) {
      return 0.0;
    }
    return std::sqrt(m2 / static_cast<double>(count));
  }
};

int parseIterations(const char *value) {
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed <= 0) {
    throw std::invalid_argument("n_iterations must be a positive integer.");
  }
  return static_cast<int>(parsed);
}

std::string buildOutputFileName(std::string_view mode, int iterations) {
  std::ostringstream fileName;
  fileName << "benchmark_" << mode << '_' << iterations << ".csv";
  return fileName.str();
}

void runBenchmarks(std::string_view mode, int iterations) {
  std::ofstream csvFile(buildOutputFileName(mode, iterations));
  if (!csvFile.is_open()) {
    throw std::runtime_error("Unable to open CSV output file.");
  }

  auto writeLine = [&](const std::string &line) {
    std::cout << line << '\n';
    csvFile << line << '\n';
  };

  writeLine("mode: " + std::string(mode));
  writeLine("iterations: " + std::to_string(iterations));
  writeLine("boids,total_ms,avg_ms,stddev_ms");

  for (int boidCount : kBoidCounts) {
    std::srand(0);

    auto runner = benchmark::createBenchmarkRunner(mode, boidCount);
    runner->init();

    RunningStats stats;
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const auto start = std::chrono::steady_clock::now();
      runner->simulateStep();
      const auto end = std::chrono::steady_clock::now();

      const double iterationMs =
          std::chrono::duration<double, std::milli>(end - start).count();
      stats.add(iterationMs);
    }
    const double totalMs = stats.total;
    const double averageMs = stats.mean;
    const double standardDeviationMs = stats.standardDeviation();

    std::ostringstream line;
    line << std::fixed << std::setprecision(3) << boidCount << ',' << totalMs
         << ',' << averageMs << ',' << standardDeviationMs;
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
    const std::string_view mode = argv[1];
    const int iterations = parseIterations(argv[2]);
    runBenchmarks(mode, iterations);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
