#include <iostream>

#include "simulation.hpp"

int main(Flock flock) {  // costruttore di flock da sistemare
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
    std::cout << "Error input";
  } else {
    std::cout << "Distance of separation (preset): " << distance_separation
              << "\nSpatial dimension (preset): " << spatial_dimension
              << "\nSimulation duration (preset): " << duration_second << "\n";

    simulation(duration_second, fps, n_boids, flock);
  }
}