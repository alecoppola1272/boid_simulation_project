#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#define _USE_MATH_DEFINES

#include <cmath>
#include <iomanip>
#include <iostream>

#include "flock.hpp"
#include "velocity_rules.hpp"

void boid_vision(std::vector<coordinates>::iterator& it1,
                 std::vector<coordinates>::iterator& it2, values const& val,
                 std::vector<std::vector<coordinates>::iterator>& neighbors) {
  double radius_direction = std::hypot(it1->v.x, it1->v.y);
  double alpha =
      std::asin(it1->v.y / radius_direction);  // prendere valore giusto dei due
  if (alpha < 0) {
    alpha += M_PI;
  }

  double radius_distance_boid =
      std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y);
  double beta = std::asin((it2->p.y - it1->p.y) / radius_distance_boid);
  if (beta < 0) {
    beta += M_PI;
  }

  if (beta < (alpha + M_PI - (M_PI * val.boid_vision_angle / 180)) &&
      beta > (alpha + M_PI + (M_PI * val.boid_vision_angle / 180))) {
    neighbors.push_back(it2);
  }
}

void checking_neighbors(
    Flock& flock, std::vector<coordinates>::iterator& it1, values const& val,
    std::vector<std::vector<coordinates>::iterator>& neighbors) {
  for (auto it2 = flock.begin(); it2 != std::prev(flock.end()); ++it2) {
    if (it2 != it1 && std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y) <=
                          val.distance_neighbors) {
      boid_vision(it1, it2, val, neighbors);
    }
  }
}

void update_velocity(Flock& flock, values const& val) {
  for (auto it1 = flock.begin(); it1 != std::prev(flock.end()); ++it1) {
    std::vector<std::vector<coordinates>::iterator> neighbors;
    checking_neighbors(flock, it1, val, neighbors);

    velocity_sum(neighbors, it1, val);
    velocity_limit(it1, val);
  }
}

void update_position(Flock& flock, int const& fps) {
  for (auto it = flock.begin(); it != std::prev(flock.end()); ++it) {
    it->p.x += it->v.x / fps;
    it->p.y += it->v.y / fps;
  }
}

void update_flock(Flock& flock, values const& val) {
  update_velocity(flock, val);
  update_position(flock, val.fps);
}

void simulation(values const& val) {
  Flock flock{{}};
  position cm{};
  velocity vm{};
  position dsm{};
  double steps_tot = val.duration_second * val.fps;

  flock.add_boids(val);

  std::cout << "Step |    vm x |    vm y |    cm x |    cm y |   dsm "
               "x |   dsm y"
            << std::endl;
  std::cout
      << "---------------------------------------------------------------- "
      << std::endl;
  for (int steps = 0; steps != steps_tot; ++steps) {
    update_flock(flock, val);

    cm = flock.center_mass(val.n_boids);
    vm = flock.velocity_mean(val.n_boids);
    dsm = flock.d_separation_mean();

    if ((steps + 1) % val.visual_steps == 0 || steps == 0) {
      std::cout << std::fixed << std::setprecision(2) << std::setw(4)
                << steps + 1 << " | " << std::setw(val.precision_output) << vm.x
                << " | " << std::setw(val.precision_output) << vm.y << " | "
                << std::setw(val.precision_output) << cm.x << " | "
                << std::setw(val.precision_output) << cm.y << " | "
                << std::setw(val.precision_output) << dsm.x << " | "
                << std::setw(val.precision_output) << dsm.y << std::endl;
      // SFML
      // cin e cout nel main
    }
  }
  std::cout << "\n";
}

#endif