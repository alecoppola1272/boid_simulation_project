#include "boids.hpp"

auto creation_flock(int n_boids) {
  // Flock flock_create;
  // auto it_last = std::prev(flock_create.end());
  // for (auto it = flock_create.begin(); it != it_last; ++it) {
  //   for (int i = 0; i != spatial_dimension; ++i) {
  //     it->velocity[i] = std::rand() % 1, 10;
  //     it->position[i] = std::rand() % 1, 99;
  //   }
  // }
}

auto evolve_flock() {
  // controllo dei vicini
  // sommare velocità a boid generati
  // limite velocità massima
}

auto simulation(double duration, int fps) {
  double steps_tot = duration * fps;
  for (int steps = 0; steps != steps_tot; ++steps) {
    // evolve_flock
  }
}
