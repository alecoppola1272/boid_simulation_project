#include <array>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include "simulation.hpp"

struct values {
  int n_boids;
  double separation_factor;
  double alignment_factor;
  double coesion_factor;
  
  int const box_length{100};
  int const edge_lenght{10};
  double const velocity_default{10.};
  double const velocity_max{20.};
  double const distance_separation{5.};
  double const edge_factor{0.1};
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
    std::cout << "Distance of separation (preset): " << val.distance_separation
              << "\nSimulation duration (preset): " << duration_second << "\n";

    simulation(duration_second, fps, val);
  }
}