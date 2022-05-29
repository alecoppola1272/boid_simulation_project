#include "boids.hpp"
#include "evolve.hpp"

#include <iostream>

int main() {
  int n_boids;

  std::cout << "Number of boids: ";
  std::cin >> n_boids;
  std::cout << "Separation factor: ";
  std::cin >> separation_factor;
  std::cout << "Alignment factor (a < 1): ";
  std::cin >> alignment_factor;
  std::cout << "Coesion factor: ";
  std::cin >> coesion_factor;
  // aggiungere regole di input
}