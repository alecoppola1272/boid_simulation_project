#include <SFML/Graphics.hpp>
#include <iomanip>
#include <iostream>

#include "evolve.hpp"

void simulation(values const& val) {
  Flock flock{{}};
  flock.add_boids(val);
  double steps_tot = val.duration_second * val.fps;

  sf::RenderWindow window(sf::VideoMode(val.box_length, val.box_length),
                          "Boids Simulation");
  window.setFramerateLimit(val.fps);
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
      update_flock(flock, val);

      if ((steps + 1) % val.visual_steps == 0 || steps == 0) {
        coordinates cm = flock.center_mass(val.n_boids);
        coordinates vm = flock.velocity_mean(val.n_boids);
        coordinates dsm = flock.d_separation_mean();
        window.draw(circle);
      }
    }
    window.display();
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
  } else {
    std::cout << "Edge factor (preset): " << val.edge_factor
              << "\nDistance of separation (preset): "
              << val.distance_separation
              << "\nDistance of neighbors (preset): " << val.distance_neighbors
              << "\nSimulation duration (preset): " << val.duration_second
              << "\n\n";

    simulation(val);
  }
}

// test, sfml, accumulate
// 40 0.7 0.4 0.7