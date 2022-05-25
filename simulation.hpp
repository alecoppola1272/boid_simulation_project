#include "flock.hpp"

int creation_birds() {
  int n_boids;
  std::cout << "Separation factor: ";
  std::cin >> separation_factor;
  std::cout << "Alignment factor (a < 1): ";
  std::cin >> alignment_factor;
  std::cout << "Coesion factor: ";
  std::cin >> coesion_factor;
  std::cout << "Number of boids: ";
  std::cin >> n_boids;
  // aggiungere regole di input

  std::vector<Flock> birds(n_boids);

  auto it = birds.begin();
  auto it_next = std::next(it);
  auto it_last = std::prev(birds.end());

  for (; it != it_last; ++it, ++it_next) {
    birds.vx = std::rand() % 10;
    birds.vy = std::rand() % 10;
    birds.px = std::rand() % 100;
    birds.py = std::rand() % 100;
  }
}