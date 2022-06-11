#ifndef MAIN_CPP
#define MAIN_CPP

#include "flock.hpp"
#include "velocity_rules.hpp"

auto evolve_flock(int fps, int n_boids) {
  Flock flock{{}};

  flock.add_boids(n_boids);
  std::vector<int> controllo_vicini;
  auto it_last = std::prev(flock.fend());

  // checking neighbors
  for (auto it1 = flock.fbegin(); it1 != it_last; ++it1) {
    for (auto it2 = flock.fbegin(); it2 != it_last; ++it2) {
      if (it1 != it2) {
        for (int i = 0; i != spatial_dimension; ++i) {
          if (std::abs(it1->position[i] - it2->position[i]) <
              separation_factor) {  // segnare it2 vicino it1
            controllo_vicini.push_back(it2);
          }
        }
      }
    }
    // velocity_sum();
  }

  for (auto it = flock.fbegin(); it != it_last; ++it) {
    for (int i = 0; i != spatial_dimension; ++i) {
      it->position[i] += it->velocity[i] / fps;
    }
  }

  return flock;
}

auto simulation(double duration, int fps, int n_boids) {
  double steps_tot = duration * fps;
  for (int steps = 0; steps != steps_tot; ++steps) {
    evolve_flock(fps, n_boids);
  }
}

#endif