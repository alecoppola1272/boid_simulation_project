#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

struct values {
  int n_boids;
  double separation_factor;
  double alignment_factor;
  double coesion_factor;

  int const box_length{100};
  int const edge_lenght{10};
  int const visual_steps{50};
  double const velocity_default{5.};
  double const velocity_max{10.};
  double const distance_neighbors{10.};
  double const distance_separation{1.};
  double const edge_factor{0.5};
  double const boid_vision{30.};
};

#include "simulation.hpp"

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

  if (val.alignment_factor >= 1) {
    throw std::runtime_error{"Error input"};
  } else {
    std::cout << "Distance of separation (preset): " << val.distance_separation
              << "\nSimulation duration (preset): " << duration_second
              << "\n\n";

    simulation(val, duration_second, fps);
  }
}

/* test, sfml, accumulate, vista boid
50 0.1 0.1 0.1
*/