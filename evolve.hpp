#ifndef EVOLVE_HPP
#define EVOLVE_HPP
#define _USE_MATH_DEFINES

#include <cmath>

#include "flock.hpp"
#include "velocity_rules.hpp"

struct val_simulation {
  int const visual_steps{50};
  int const precision_output{7};
  double const duration_second{20.0};
  int const fps{30};
};

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

  if (beta < (alpha + M_PI - (M_PI * val.boid_vision_angle / 360)) ||
      beta > (alpha + M_PI + (M_PI * val.boid_vision_angle / 360))) {
    neighbors.push_back(it2);
  }
}

void checking_neighbors(Flock& flock, std::vector<boid>::iterator& it1,
                        values const& val,
                        std::vector<std::vector<boid>::iterator>& neighbors) {
  for (auto it2 = flock.begin(); it2 != flock.end(); ++it2) {
    if (it2 != it1 && std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y) <=
                          val.distance_neighbors) {
      boid_vision(it1, it2, val, neighbors);
    }
  }
}

void update_velocity(Flock& flock, values const& val) {
  for (auto it1 = flock.begin(); it1 != flock.end(); ++it1) {
    std::vector<std::vector<boid>::iterator> neighbors;
    checking_neighbors(flock, it1, val, neighbors);

    if (neighbors.empty() == 0) {
      velocity_sum(neighbors, it1, val);
    }

    it1->v = it1->v + velocity_edge(it1, val);
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

void update_position(Flock& flock, values const& val, val_simulation const& sim) {
  for (auto it = flock.begin(); it != flock.end(); ++it) {
    it->p = it->p + (it->v / sim.fps);

    position_limit(it, val);
  }
}

void update_flock(Flock& flock, values const& val, val_simulation const& sim) {
  update_velocity(flock, val);
  update_position(flock, val, sim);
}

#endif