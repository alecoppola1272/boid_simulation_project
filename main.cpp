#include <iomanip>
#include <iostream>

struct val_simulation {
  int const visual_steps{50};
  int const precision_output{7};
  double const duration_second{20.0};
  int const fps{30};
};

#include "evolve.hpp"

void simulation(values const& val, val_simulation const& sim) {
  Flock flock{{}};
  flock.add_boids(val);
  double steps_tot = sim.duration_second * sim.fps;

  std::cout << "Step |    vm x    vm y |   dsm "
               "x   dsm y |    cm x    cm y"
            << std::endl;
  std::cout << "---------------------------------------------------------- "
            << std::endl;

  for (int steps = 0; steps != steps_tot; ++steps) {
    update_flock(flock, val, sim);

    if ((steps + 1) % sim.visual_steps == 0 || steps == 0) {
      coordinates vm = flock.velocity_mean(val.n_boids);
      coordinates dsm = flock.d_separation_mean();
      std::cout << std::fixed << std::setprecision(2) << std::setw(4)
                << steps + 1 << " | " << std::setw(sim.precision_output) << vm.x
                << " " << std::setw(sim.precision_output) << vm.y << " | "
                << std::setw(sim.precision_output) << dsm.x << " "
                << std::setw(sim.precision_output) << dsm.y << std::endl;
    }
  }
  std::cout << "\n";
}

int main() {
  values val;
  val_simulation sim;

  std::cout << "Number of boids (n > 0): ";
  std::cin >> val.n_boids;
  std::cout << "Separation factor (s > 0): ";
  std::cin >> val.separation_factor;
  std::cout << "Alignment factor (0 < a < 1): ";
  std::cin >> val.alignment_factor;
  std::cout << "Coesion factor (c > 0): ";
  std::cin >> val.coesion_factor;

  if (isdigit(val.separation_factor) || isdigit(val.alignment_factor) ||
      isdigit(val.coesion_factor) || val.n_boids <= 0 ||
      val.separation_factor <= 0 || val.alignment_factor <= 0 ||
      val.alignment_factor >= 1 || val.coesion_factor <= 0) {
    throw std::runtime_error{"Input error"};
  }

  std::cout << "Edge factor (preset): " << val.edge_factor
            << "\nDistance of separation (preset): " << val.distance_separation
            << "\nDistance of neighbors (preset): " << val.distance_neighbors
            << "\nSimulation duration (preset): " << sim.duration_second
            << "\n\n";

  std::cout << val.n_boids << "\n"
            << val.separation_factor << "\n"
            << val.alignment_factor << "\n"
            << val.coesion_factor << "\n";

  simulation(val, sim);
}

// test, accumulate, errore val_sim, v_rules in flock, sfml
// 30 .1 .1 .1