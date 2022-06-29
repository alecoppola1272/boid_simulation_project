#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#define _USE_MATH_DEFINES

#include <cmath>
#include <iomanip>
#include <iostream>

#include "flock.hpp"
#include "velocity_rules.hpp"

void dial_control(double const y, double const x, double& angle) {
  if (y >= 0 && x < 0) {
    angle = M_PI - angle;
  } else if (y < 0 && x >= 0) {
    angle += 2 * M_PI;
  } else if (y < 0 && x < 0) {
    angle = M_PI - angle;
  }
}

void boid_vision(std::vector<boid>::iterator& it1,
                 std::vector<boid>::iterator& it2, values const& val,
                 std::vector<std::vector<boid>::iterator>& neighbors) {
  coordinates a = it1->v / std::hypot(it1->v.x, it1->v.y);
  coordinates b =
      (it2->p - it1->p) / std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y);

  double alpha = std::asin(a.y);
  double beta = std::asin(b.y);

  dial_control(a.y, a.x, alpha);
  dial_control(b.y, b.x, beta);

  if (alpha > beta) {
    beta += 2 * M_PI;
  }

  if (beta < (alpha + M_PI - (M_PI * val.boid_vision_angle / 180)) ||
      beta > (alpha + M_PI + (M_PI * val.boid_vision_angle / 180))) {
    neighbors.push_back(it2);
  }
}

void checking_neighbors(Flock& flock, std::vector<boid>::iterator& it1,
                        values const& val,
                        std::vector<std::vector<boid>::iterator>& neighbors) {
  for (auto it2 = flock.begin(); it2 != std::prev(flock.end()); ++it2) {
    if (it2 != it1 && std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y) <=
                          val.distance_neighbors) {
      boid_vision(it1, it2, val, neighbors);
    }
  }
}

void update_velocity(Flock& flock, values const& val) {
  for (auto it1 = flock.begin(); it1 != std::prev(flock.end()); ++it1) {
    std::vector<std::vector<boid>::iterator> neighbors;
    checking_neighbors(flock, it1, val, neighbors);

    velocity_sum(neighbors, it1, val);
    velocity_limit(it1, val);
  }
}

void position_limit(std::vector<boid>::iterator& it, values const& val) {
  if (it->p.x < 0) {
    it->p.x = 0;
  } else if (it->p.x > val.box_length) {
    it->p.x = val.box_length;
  }

  if (it->p.y < 0) {
    it->p.y = 0;
  } else if (it->p.y > val.box_length) {
    it->p.y = val.box_length;
  }
}

void update_position(Flock& flock, values const& val) {
  for (auto it = flock.begin(); it != std::prev(flock.end()); ++it) {
    it->p = it->p + (it->v / val.fps);

    position_limit(it, val);
  }
}

void update_flock(Flock& flock, values const& val) {
  update_velocity(flock, val);
  update_position(flock, val);
}

void simulation(values const& val) {
  Flock flock{{}};
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

    if ((steps + 1) % val.visual_steps == 0 || steps == 0) {
      coordinates cm = flock.center_mass(val.n_boids);
      coordinates vm = flock.velocity_mean(val.n_boids);
      coordinates dsm = flock.d_separation_mean();
      std::cout << std::fixed << std::setprecision(2) << std::setw(4)
                << steps + 1 << " | " << std::setw(val.precision_output) << vm.x
                << " | " << std::setw(val.precision_output) << vm.y << " | "
                << std::setw(val.precision_output) << cm.x << " | "
                << std::setw(val.precision_output) << cm.y << " | "
                << std::setw(val.precision_output) << dsm.x << " | "
                << std::setw(val.precision_output) << dsm.y << std::endl;
      // SFML
      // iostream in main
    }
  }
  std::cout << "\n";
}

#endif