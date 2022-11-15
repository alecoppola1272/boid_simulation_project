#include <iomanip>
#include <iostream>

#include "flock.hpp"

struct val_simulation {
  int const steps{10000};
  int const visual_steps{500};
  int const precision_output{7};
};

void simulation(values const& val, val_simulation const& sim) {
  Flock flock;
  flock.add_boids(val);

  auto fbegin = flock.begin();
  auto fend = flock.end();

  std::cout << "Step |      vm     std |     dsm     std |    cm x    cm y   "
               "vcm x   vcm y"
            << '\n';
  std::cout << "---------------------------------------------------------------"
               "-----------"
            << '\n';

  for (int steps = 0; steps != sim.steps; ++steps) {
    flock.update_flock(flock, val, 60, steps);

    if ((steps + 1) % sim.visual_steps == 0 || steps == 0) {
      float vm = flock.velocity_mean(val.n_boids);
      std::vector<float> hypotenuse_v{};
      for (auto it = fbegin; it != fend; ++it) {
        hypotenuse_v.push_back(std::hypot(it->v.x, it->v.y));
      };
      float std_vm = stddev(hypotenuse_v);

      float dsm = flock.d_separation_mean();
      std::vector<float> hypotenuse_ds{};
      for (auto it1 = fbegin; it1 != fend; ++it1) {
        for (auto it2 = std::next(it1); it2 != fend; ++it2) {
          hypotenuse_ds.push_back(
              std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y));
        }
      };
      float std_dsm = stddev(hypotenuse_ds);

      coordinates cm = flock.center_mass(val.n_boids);
      coordinates vcm = flock.velocity_cm(val.n_boids);

      std::cout << std::fixed << std::setprecision(2) << std::setw(4)
                << steps + 1 << " | " << std::setw(sim.precision_output) << vm
                << " " << std::setw(sim.precision_output) << std_vm << " | "
                << std::setw(sim.precision_output) << dsm << " "
                << std::setw(sim.precision_output) << std_dsm << " | "
                << std::setw(sim.precision_output) << cm.x << " "
                << std::setw(sim.precision_output) << cm.y << " "
                << std::setw(sim.precision_output) << vcm.x << " "
                << std::setw(sim.precision_output) << vcm.y << '\n';
    }
  }
  std::cout << std::endl;
}

int main() {
  values val;
  val_simulation sim;
  std::cout << "Boid simulation\n\n";
  std::cout << "Number of boids (n > 1): ";
  std::cin >> val.n_boids;
  std::cout << "Separation factor (s > 0): ";
  std::cin >> val.separation_factor;
  std::cout << "Alignment factor (0 < a < 1): ";
  std::cin >> val.alignment_factor;
  std::cout << "Coesion factor (c > 0): ";
  std::cin >> val.coesion_factor;

  if (val.n_boids <= 1 || val.separation_factor <= 0 ||
      val.alignment_factor <= 0 || val.alignment_factor >= 1 ||
      val.coesion_factor <= 0) {
    throw std::runtime_error{"Input error"};
  }

  std::cout << "\n- Preset values -"
            << "\nEdge factor: " << val.edge_factor
            << "\nBox lenght: " << val.box_length
            << "\nEdge lenght: " << val.edge_lenght
            << "\nVelocity default: " << val.velocity_default
            << "\nVelocity max: " << val.velocity_max
            << "\nVelocity min: " << val.velocity_min
            << "\nVelocity balancer: " << val.velocity_balancer
            << "\nDistance neighbors: " << val.distance_neighbors
            << "\nDistance separation: " << val.distance_separation
            << "\nBoid blind angle: " << val.boid_blind_angle
            << "\nSteps: " << sim.steps
            << "\nVisual steps: " << sim.visual_steps
            << "\nPrecision output: " << sim.precision_output << "\n\n";

  simulation(val, sim);
}