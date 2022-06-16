#include <array>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include "simulation.hpp"

constexpr int box_length{100};
constexpr int edge_lenght{10};
constexpr double velocity_default{10.};
constexpr double velocity_max{20.};
constexpr double distance_neighbors{5.};
constexpr double distance_separation{5.};
constexpr double edge_factor{0.1};

struct values {
  int n_boids;
  double separation_factor;
  double alignment_factor;
  double coesion_factor;
};

int main() {
  values val;
  double const duration_second{20.0};
  int const fps{30};

  std::cout << "Number of boids: ";
  std::cin >> val.n_boids;
  std::cout << "Separation factor: ";
  std::cin >> val.separation_factor;
  std::cout << "Alignment factor (a < 1): ";
  std::cin >> val.alignment_factor;
  std::cout << "Coesion factor: ";
  std::cin >> val.coesion_factor;

  if (val.alignment_factor > 1) {
    std::cout << "Error input";  // runtime_ERROR (throw)
  } else {
    std::cout << "Distance of separation (preset): " << distance_separation
              << "\nSimulation duration (preset): " << duration_second << "\n";

    simulation(duration_second, fps, val.n_boids, val);
  }
}