#include <SFML/Graphics.hpp>
#include <iomanip>
#include <iostream>

struct val_simulation {
  int const visual_steps{50};
  double const duration_second{20.0};
  int const fps{30};
};

#include "evolve.hpp"

void simulation(values const& val, val_simulation const& sim) {
  Flock flock{{}};
  flock.add_boids(val);
  double steps_tot = sim.duration_second * sim.fps;

  //   sf::RenderWindow window(sf::VideoMode(val.box_length, val.box_length),
  //   "Flock simulation"); sf::Time time_duration =
  //   sf::seconds(val.duration_second); sf::Clock clock; sf::Time t =
  //   clock.restart();
  //   //   sf::Time elapsed1 = clock.getElapsedTime();
  //   //   std::cout << elapsed1.asSeconds() << std::endl;
  //   while (window.isOpen()) {
  //     sf::Time elapsed = clock.restart();
  //   }

  sf::RenderWindow window(sf::VideoMode(val.box_length, val.box_length),
                          "Boids Simulation");
  window.setFramerateLimit(sim.fps);
  sf::CircleShape circle(val.distance_separation);
  circle.setFillColor(sf::Color::Yellow);

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    window.clear(sf::Color::White);

    for (int steps = 0; steps != steps_tot; ++steps) {
      update_flock(flock, val, sim);
      window.draw(circle);
    }
    window.display();
  }
}

int main() {
  values val;
  val_simulation sim;

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
            << "\nSimulation duration (preset): " << sim.duration_second
            << "\n\n";

  simulation(val, sim);
}

// test, sfml, accumulate
// 30 .1 .1 .1