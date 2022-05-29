#include "boids.hpp"

constexpr double distance_max;

void creation_b(double x) {
  std::vector<boid> birds(x);
  Flock flock_create{birds, distance_max};

  auto it_last = std::prev(birds.end());
  for (auto it = birds.begin(); it != it_last; ++it) {
    it->vx = std::rand() % 10;
    it->vy = std::rand() % 10;
    it->px = std::rand() % 100;
    it->py = std::rand() % 100;
  }

  for (auto it = birds.begin(); it != it_last; ++it) {
  }
}