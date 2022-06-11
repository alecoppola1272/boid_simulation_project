#include <array>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

#include "simulation.hpp"

constexpr int spatial_dimension{2};
constexpr int box_length{100};
constexpr int edge_lenght{10};
constexpr double velocity_default{10.};
constexpr double velocity_max{20.};
constexpr double distance_neighbors{5.};
constexpr double distance_separation{5.};
constexpr double edge_factor{0.1};
double separation_factor;
double alignment_factor;
double coesion_factor;

int main() {
  int n_boids;
  double const duration_second{20.0};
  int const fps{30};

  std::cout << "Number of boids: ";
  std::cin >> n_boids;
  std::cout << "Separation factor: ";
  std::cin >> separation_factor;
  std::cout << "Alignment factor (a < 1): ";
  std::cin >> alignment_factor;
  std::cout << "Coesion factor: ";
  std::cin >> coesion_factor;

  if (alignment_factor > 1) {
    std::cout << "Error input";  // runtime_ERROR (throw)
  } else {
    std::cout << "Distance of separation (preset): " << distance_separation
              << "\nSpatial dimension (preset): " << spatial_dimension
              << "\nSimulation duration (preset): " << duration_second << "\n";

    simulation(duration_second, fps, n_boids);
  }
}