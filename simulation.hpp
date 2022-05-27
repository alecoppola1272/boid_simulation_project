#include "flock.hpp"

constexpr double distance_max;

void creation_birds(double x) {
  std::vector<boid> birds(x);
  Flock flock_create{birds, distance_max};

  auto it = birds.begin();
  auto it_next = std::next(it);
  auto it_last = std::prev(birds.end());

  for (; it != it_last; ++it, ++it_next) {
    it->vx = std::rand() % 10;
    it->vy = std::rand() % 10;
    it->px = std::rand() % 100;
    it->py = std::rand() % 100;
  }
}