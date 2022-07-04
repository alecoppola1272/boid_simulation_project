#include <iomanip>
#include <iostream>

#include "evolve.hpp"

void simulation(values const& val) {
  Flock flock{{}};
  flock.add_boids(val);
  double steps_tot = val.duration_second * val.fps;

  std::cout << "Step |    vm x    vm y |    cm x    cm y |   dsm "
               "x   dsm y"
            << std::endl;
  std::cout << "---------------------------------------------------------- "
            << std::endl;

  for (int steps = 0; steps != steps_tot; ++steps) {
    update_flock(flock, val);

    if ((steps + 1) % val.visual_steps == 0 || steps == 0) {
      coordinates cm = flock.center_mass(val.n_boids);
      coordinates vm = flock.velocity_mean(val.n_boids);
      coordinates dsm = flock.d_separation_mean();
      std::cout << std::fixed << std::setprecision(2) << std::setw(4)
                << steps + 1 << " | " << std::setw(val.precision_output) << vm.x
                << " " << std::setw(val.precision_output) << vm.y << " | "
                << std::setw(val.precision_output) << cm.x << " "
                << std::setw(val.precision_output) << cm.y << " | "
                << std::setw(val.precision_output) << dsm.x << " "
                << std::setw(val.precision_output) << dsm.y << std::endl;
      // SFML
    }
  }
  std::cout << "\n";
}

int main() {
  values val;

  std::cout << "Number of boids: ";
  std::cin >> val.n_boids;
  std::cout << "Separation factor (s > 0): ";
  std::cin >> val.separation_factor;
  std::cout << "Alignment factor (0 < a < 1): ";
  std::cin >> val.alignment_factor;
  std::cout << "Coesion factor (c > 0): ";
  std::cin >> val.coesion_factor;

  if (val.n_boids <= 0 || val.separation_factor <= 0 ||
      val.alignment_factor >= 1 || val.alignment_factor <= 0 ||
      val.coesion_factor <= 0) {
    throw std::runtime_error{"Input error"};
  } else {
    std::cout << "Edge factor (preset): " << val.edge_factor
              << "\nDistance of separation (preset): "
              << val.distance_separation
              << "\nDistance of neighbors (preset): " << val.distance_neighbors
              << "\nSimulation duration (preset): " << val.duration_second
              << "\n\n";

    simulation(val);
  }
}
