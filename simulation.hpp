#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "flock.hpp"
#include "velocity_rules.hpp"

auto checking_neighbors(int n_boids, Flock flock, values value) {
  auto it_last = std::prev(flock.end());

  for (auto it1 = flock.begin(); it1 != it_last; ++it1) {
    std::vector<std::vector<coordinates>::iterator> controllo_vicini;

    for (auto it2 = flock.begin(); it2 != it_last; ++it2) {
      if (it1 != it2) {
        if (std::abs(it1->p.x - it2->p.x) < value.separation_factor &&
            std::abs(it1->p.y - it2->p.y) < value.separation_factor) {
          controllo_vicini.push_back(it2);  // segnare it2 vicino it1
        }
      }
    }
    update_velocity(flock, it1, value, n_boids, controllo_vicini);  // !
  }

  return flock;
}

auto update_velocity(Flock flock, auto it, values value, int n_boids,
                     auto controllo_vicini) {
  velocity v_sum{0., 0.};
  v_sum = velocity_sum(v_sum, flock, it, value, n_boids, controllo_vicini);
  v_sum = velocity_limit(v_sum);

  return v_sum;
}

auto update_position(Flock flock, int fps) {
  auto it_last = std::prev(flock.end());

  for (auto it = flock.begin(); it != it_last; ++it) {
    it->p.x += it->v.x / fps;
    it->p.y += it->v.y / fps;
  }

  return flock;
}

auto evolve_flock(int fps, int n_boids, Flock flock, values value) {
  flock.add_boids(n_boids);
  flock = checking_neighbors(n_boids, flock, value);
  flock = update_position(flock, fps);

  return flock;
}

auto simulation(double duration, int fps, int n_boids, values value) {
  Flock flock{{}};
  double steps_tot = duration * fps;

  for (int steps = 0; steps != steps_tot; ++steps) {
    flock = evolve_flock(fps, n_boids, flock, value);
    // sfml
  }
}

#endif