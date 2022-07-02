#include <SFML/Graphics.hpp>
#include <iomanip>
#include <iostream>

struct values {
  int n_boids{};
  double separation_factor{};
  double alignment_factor{};
  double coesion_factor{};

  double const edge_factor{0.1};
  int const box_length{500};
  int const edge_lenght{10};

  int const visual_steps{10};
  int const precision_output{7};

  double const velocity_default{10.};
  double const velocity_max{20.};

  double const distance_neighbors{10.};
  double const distance_separation{2.};
  double const boid_vision_angle{30.};

  double const duration_second{20.0};
  int const fps{30};
};

#include "evolve.hpp"

void simulation(values const& val) {
  Flock flock{{}};
  flock.add_boids(val);
  double steps_tot = val.duration_second * val.fps;

  //   sf::RenderWindow window(sf::VideoMode(val.box_length, val.box_length),
  //   "Flock simulation"); sf::Time time_duration =
  //   sf::seconds(val.duration_second); sf::Clock clock; sf::Time t =
  //   clock.restart();
  //   //   sf::Time elapsed1 = clock.getElapsedTime();
  //   //   std::cout << elapsed1.asSeconds() << std::endl;
  //   while (window.isOpen()) {
  //     sf::Time elapsed = clock.restart();
  //   }

  for (int steps = 0; steps != steps_tot; ++steps) {
    update_flock(flock, val);
  }
}

int main() {
  values val;

  std::cout << "Number of boids (n > 0): ";
  std::cin >> val.n_boids;
  std::cout << "Separation factor (s > 0): ";
  std::cin >> val.separation_factor;
  std::cout << "Alignment factor (0 < a < 1): ";
  std::cin >> val.alignment_factor;
  std::cout << "Coesion factor (c > 0): ";
  std::cin >> val.coesion_factor;

  if (isdigit(val.n_boids) || isdigit(val.separation_factor) ||
      isdigit(val.alignment_factor) || isdigit(val.coesion_factor) ||
      val.n_boids <= 0 || val.separation_factor <= 0 ||
      val.alignment_factor >= 1 || val.alignment_factor <= 0 ||
      val.coesion_factor <= 0) {
    throw std::runtime_error{"Input error"};
  }
  
  std::cout << "Edge factor (preset): " << val.edge_factor
            << "\nDistance of separation (preset): " << val.distance_separation
            << "\nDistance of neighbors (preset): " << val.distance_neighbors
            << "\nSimulation duration (preset): " << val.duration_second
            << "\n\n";

  simulation(val);
}

// test, sfml, accumulate
// 30 .1 .1 .1