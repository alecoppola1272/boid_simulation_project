#include <SFML/Graphics.hpp>
#include <iomanip>
#include <iostream>

#include "flock.hpp"
#include "evolve.hpp"

struct val_simulation {
  float const duration_second{60};
  int const fps{60};
  float const boid_dimension{8};
  int const visual_steps{fps};
  int const precision_output{7};
};

void simulation(values const& val, val_simulation const& sim) {
  Flock flock;
  flock.add_boids(val);
  sf::ContextSettings settings;
  settings.antialiasingLevel = 5;

  sf::RenderWindow window(sf::VideoMode(val.box_length, val.box_length),
                          "Flock simulation", sf::Style::Default, settings);
  window.setFramerateLimit(sim.fps);

  sf::VertexArray boid(sf::TrianglesFan, 4);
  boid[0].color = sf::Color::White;
  boid[1].color = sf::Color::White;
  boid[2].color = sf::Color::White;  // top
  boid[3].color = sf::Color::White;

  while (window.isOpen()) {
    float steps_tot = sim.duration_second * sim.fps;
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    std::cout << "Secondi |      vm     std |     dsm     std |    cm x    cm "
                 "y   vcm x   vcm y"
              << '\n';
    std::cout << "-------------------------------------------------------------"
                 "----------------"
              << '\n';

    Flock flock_smooth{flock};
    auto fbegin = flock.begin();
    auto fend = flock.end();
    auto fsbegin = flock_smooth.begin();
    auto fsend = flock_smooth.end();
    for (int step = 0; step != steps_tot; ++step) {
      flock.update_flock(flock, val, sim.fps, step);
      flock_smooth = flock;
      if (val.velocity_step_update > 2 &&
          step % val.velocity_step_update != 0) {
        update_velocity(flock_smooth, val);
        for (auto it_smooth = fsbegin, it = fbegin; it_smooth != fsend;
             ++it, ++it_smooth) {
          if (it_smooth->v.x + it->v.x != 0 && it_smooth->v.y + it->v.y != 0) {
            it_smooth->v = (it_smooth->v + it->v) / 2;
          }
        }
      }

      if ((step + 1) % sim.visual_steps == 0 || step == 0) {
        float vm = flock.velocity_mean(val.n_boids);
        std::vector<float> hypotenuse_v{};
        for (auto it = fbegin; it != fend; ++it) {
          hypotenuse_v.push_back(std::hypot(it->v.x, it->v.y));
        };
        float std_vm = stddev(hypotenuse_v);

        float dsm = flock.d_separation_mean();
        std::vector<float> hypotenuse_ds{};
        for (auto it1 = fbegin; it1 != fend; ++it1) {
          for (auto it2 = std::next(it1); it2 != fend; ++it2) {
            hypotenuse_ds.push_back(
                std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y));
          }
        };
        float std_dsm = stddev(hypotenuse_ds);
        coordinates cm = flock.center_mass(val.n_boids);
        coordinates vcm = flock.velocity_cm(val.n_boids);

        std::cout << std::fixed << std::setprecision(2)
                  << std::setw(sim.precision_output) << (step + 1) / sim.fps
                  << " | " << std::setw(sim.precision_output) << vm << " "
                  << std::setw(sim.precision_output) << std_vm << " | "
                  << std::setw(sim.precision_output) << dsm << " "
                  << std::setw(sim.precision_output) << std_dsm << " | "
                  << std::setw(sim.precision_output) << cm.x << " "
                  << std::setw(sim.precision_output) << cm.y << " "
                  << std::setw(sim.precision_output) << vcm.x << " "
                  << std::setw(sim.precision_output) << vcm.y << '\n';
      }
      for (auto it = fsbegin; it != fsend; ++it) {
        coordinates a{it->v.x / std::hypot(it->v.x, it->v.y),
                      it->v.y / std::hypot(it->v.x, it->v.y)};

        boid[0].position =
            sf::Vector2f(it->p.x - (sim.boid_dimension * 5 / 8 * a.x),
                         it->p.y - (sim.boid_dimension * 5 / 8 * a.y));
        boid[1].position =
            sf::Vector2f(it->p.x - (sim.boid_dimension * a.x),
                         it->p.y + (sim.boid_dimension / 2 * a.y));
        boid[2].position = sf::Vector2f(it->p.x + (sim.boid_dimension * a.x),
                                        it->p.y + (sim.boid_dimension * a.y));
        boid[3].position =
            sf::Vector2f(it->p.x - (sim.boid_dimension * a.x),
                         it->p.y - (sim.boid_dimension / 2 * a.y));

        window.draw(boid);
      }

      window.display();
      window.clear(sf::Color(20, 20, 20));
    }
    window.close();
    std::cout << std::endl;
  }
}

int main() {
  values val;
  val_simulation sim;
  std::cout << "Boid simulation\n\n";
  std::cout << "Number of boids (n > 1): ";
  std::cin >> val.n_boids;
  std::cout << "Separation factor (s > 0): ";
  std::cin >> val.separation_factor;
  std::cout << "Alignment factor (0 < a < 1): ";
  std::cin >> val.alignment_factor;
  std::cout << "Coesion factor (c > 0): ";
  std::cin >> val.coesion_factor;

  if (val.n_boids <= 1 || val.separation_factor <= 0 ||
      val.alignment_factor <= 0 || val.alignment_factor >= 1 ||
      val.coesion_factor <= 0) {
    throw std::runtime_error{"Input error"};
  }

  std::cout << "\n- Preset values -"
            << "\nEdge factor: " << val.edge_factor
            << "\nBox lenght: " << val.box_length
            << "\nEdge lenght: " << val.edge_lenght
            << "\nVelocity default: " << val.velocity_default
            << "\nVelocity max: " << val.velocity_max
            << "\nVelocity min: " << val.velocity_min
            << "\nVelocity increase: " << val.velocity_balancer
            << "\nDistance neighbors: " << val.distance_neighbors
            << "\nDistance separation: " << val.distance_separation
            << "\nBoid blind angle: " << val.boid_blind_angle
            << "\nDuration second: " << sim.duration_second
            << "\nFps: " << sim.fps << "\nVisual steps: " << sim.visual_steps
            << "\nPrecision output: " << sim.precision_output << "\n\n";

  simulation(val, sim);
}