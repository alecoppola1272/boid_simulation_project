#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "flock.hpp"
#include "velocity_rules.hpp"

auto evolve_flock(int fps, int n_boids, Flock flock, values value) {
  flock.add_boids(n_boids);
  std::vector<std::vector<coordinates>::iterator> controllo_vicini;
  auto it_last = std::prev(flock.fend());

  // checking neighbors
  for (auto it1 = flock.fbegin(); it1 != it_last; ++it1) {
    for (auto it2 = flock.fbegin(); it2 != it_last; ++it2) {
      if (it1 != it2) {
        if (std::abs(it1->px - it2->px) < value.separation_factor &&
            std::abs(it1->py - it2->py) < value.separation_factor) {
          controllo_vicini.push_back(it2);  // segnare it2 vicino it1
        }
      }
      update_velocity(flock, it1, value, n_boids);
    }

    for (auto it = flock.fbegin(); it != it_last; ++it) {
      it->px += it->vx / fps;
      it->py += it->vy / fps;
    }
  }

  return flock;
}

auto simulation(double duration, int fps, int n_boids, values value) {
  Flock flock{{}};
  double steps_tot = duration * fps;
  for (int steps = 0; steps != steps_tot; ++steps) {
    evolve_flock(fps, n_boids, flock, value);
  }
}

#endif