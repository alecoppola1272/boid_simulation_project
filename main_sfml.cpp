#include <SFML/Graphics.hpp>
#include <iomanip>
#include <iostream>

#include "evolve.hpp"

struct values {
  // values input
  int n_boids{};
  double separation_factor{};
  double alignment_factor{};
  double coesion_factor{};

  // values box
  double const edge_factor{0.1};
  int const box_length{500};
  int const edge_lenght{10};

  // values velocity
  double const velocity_default{10.};
  double const velocity_max{20.};

  // values separation
  double const distance_neighbors{10.};
  double const distance_separation{2.};
  double const boid_vision_angle{30.};

  // values output
  int const visual_steps{50};
  int const precision_output{7};

  // values simulation
  double const duration_second{20.0};
  int const fps{30};
};

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
