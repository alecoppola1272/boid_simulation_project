#ifndef MAIN_CPP
#define MAIN_CPP

#include "boids.hpp"

auto creation_flock(int n_boids) {
  Flock flock_create[n_boids];
  auto it = flock_create.begin();
  auto it_last = std::prev(flock_create.end());

  for (; it != it_last; ++it) {
    for (int i = 0; i != spatial_dimension; ++i) {
      it->velocity[i] = std::rand() % 1, velocity_default;
      it->position[i] = std::rand() % 1, 99;
    }
  }

  return flock_create;
}

auto evolve_flock(int fps) {
  auto flock_evolve = creation_flock();

  // controllo dei vicini

  auto it = flock_create.begin();
  auto it_last = std::prev(flock_create.end());

  for (; it != it_last; ++it) {
    for (int i = 0; i != spatial_dimension; ++i) {
      it->flock_evolve.position[i] += it->flock_evolve.velocity[i] / fps;
    }
  }
  

  return flock_evolve;
}

auto simulation(double duration, int fps) {
  double steps_tot = duration * fps;
  for (int steps = 0; steps != steps_tot; ++steps) {
    // evolve_flock
  }
}

#endif