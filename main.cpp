#include <iomanip>
#include <iostream>

struct values {
  int n_boids{};
  double separation_factor{};
  double alignment_factor{};
  double coesion_factor{};

  double const edge_factor{0.1};
  int const box_length{500};
  int const edge_lenght{10};

  int const visual_steps{50};
  int const precision_output{7};

  double const velocity_default{10.};
  double const velocity_max{20.};

  double const distance_neighbors{10.};
  double const distance_separation{2.};
  double const boid_vision_angle{30.};

  double const duration_second{20.0};
  int const fps{30};
};

#include "evolve.hpp"

void simulation(values const& val) {
  Flock flock{{}};
  flock.add_boids(val);
  double steps_tot = val.duration_second * val.fps;

  std::cout << "Step |    vm x    vm y |   dsm "
               "x   dsm y |    cm x    cm y"
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
                << std::setw(val.precision_output) << dsm.x << " "
                << std::setw(val.precision_output) << dsm.y << " | "
                << std::setw(val.precision_output) << cm.x << " "
                << std::setw(val.precision_output) << cm.y << std::endl;
      // SFML
    }
  }
  std::cout << "\n";
}

int main() {
  values val;

  std::cout << "Number of boids (n > 0): ";
  std::cin >> val.n_boids;
  std::cout << "Separation factor (s > 0): ";
  std::cin >> val.separation_factor;
  std::cout << "Alignment factor (0 < a < 1): ";
  std::cin >> val.alignment_factor;
  std::cout << "Coesion factor (c > 0): ";
  std::cin >> val.coesion_factor;

  if (isdigit(val.n_boids) || isdigit(val.separation_factor) ||
      isdigit(val.alignment_factor) || isdigit(val.coesion_factor) ||
      val.n_boids <= 0 || val.separation_factor <= 0 ||
      val.alignment_factor <= 0 || val.alignment_factor >= 1 ||
      val.coesion_factor <= 0) {
    throw std::runtime_error{"Input error"};
  }

  std::cout << "Edge factor (preset): " << val.edge_factor
            << "\nDistance of separation (preset): " << val.distance_separation
            << "\nDistance of neighbors (preset): " << val.distance_neighbors
            << "\nSimulation duration (preset): " << val.duration_second
            << "\n\n";

  simulation(val);
}

// test, sfml, accumulate
// 30 .1 .1 .1