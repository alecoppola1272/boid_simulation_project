#ifndef EVOLVE_HPP
#define EVOLVE_HPP
#define _USE_MATH_DEFINES

#include <cmath>

#include "velocity_rules.hpp"

inline float dial_control(float const y, float const x, float angle) {
  if ((y >= 0 && x < 0) || (y < 0 && x < 0)) {
    angle -= M_PI;
  } else if (y < 0 && x >= 0) {
    angle += 2 * M_PI;
  }
  return angle;
}

inline bool boid_vision(boid& b1, boid& b2, float const& boid_blind_angle) {
  bool check{1};
  if (b1.v.x != 0 || b1.v.y != 0) {
    coordinates a = b1.v / std::hypot(b1.v.x, b1.v.y);
    coordinates b =
        (b2.p - b1.p) / std::hypot(b1.p.x - b2.p.x, b1.p.y - b2.p.y);

    float alpha = dial_control(a.y, a.x, std::asin(a.y));
    float beta = dial_control(b.y, b.x, std::asin(b.y));

    if (alpha > beta) {
      beta += 2 * M_PI;
    }
    if (beta > (alpha + M_PI - (M_PI * boid_blind_angle / 360)) &&
        beta < (alpha + M_PI + (M_PI * boid_blind_angle / 360))) {
      check = 0;
    }
  }
  return check;
}

inline auto checking_neighbors(
    Flock& flock, std::vector<boid>::iterator& it1,
    values const& val) {
  std::vector<std::vector<boid>::iterator> neighbors{};
  auto fbegin = flock.begin();
  auto fend = flock.end();
  for (auto it2 = fbegin; it2 != fend; ++it2) {
    if (it2 != it1 &&
        std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y) <=
            val.distance_neighbors &&
        boid_vision(*it1, *it2, val.boid_blind_angle)) {
      neighbors.push_back(it2);
    }
  }
  return neighbors;
}

inline void update_velocity(Flock& flock, values const& val) {
  auto fbegin = flock.begin();
  auto fend = flock.end();
  for (auto it1 = fbegin; it1 != fend; ++it1) {
    auto neighbors = checking_neighbors(flock, it1, val);

    if (neighbors.empty() == 0) {
      it1->v += velocity_rules_sum(neighbors, *it1, val);
    }
    it1->v += velocity_edge(*it1, val);
    it1->v *= val.velocity_balancer;
    it1->v = velocity_limit(it1->v, val);
  }
}

inline void update_position(Flock& flock, int const& fps) {
  auto fbegin = flock.begin();
  auto fend = flock.end();
  for (auto it = fbegin; it != fend; ++it) {
    it->p += (it->v / fps);
  }
}

#endif