#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

struct boids {
  double vx{};
  double vy{};
  double px{};
  double py{};
  double const ds{};  // represent the "dimension" (radius) of each boid
};
class flock {
  std::vector<boids> boid_;  // vector which represent each boid
  double d_;

 public:
  flock(std::vector<boids> boid, double d) : boid_{boid}, d_{d} {};

  float operator+() {}

  int size() const { return boid_.size(); }

  double mass_center() {
    double x_tot = 0;  // sum of x position
    double y_tot = 0;  // sum of y position
    int l = size();
    int i = 0;

    // introdurre lambda

    auto x_tot = std::accumulate(boid_.begin(), boid_.end(), 0);
    auto y_tot = std::accumulate(boid_.begin(), boid_.end(), 0);

    auto x_cm = x_tot / (l - 1);
    auto y_cm = y_tot / (l - 1);
  }
  // Regola 1 : Separazione
  double separation() {
    for (int i, j; i != j; ++i, ++j) {
      double vx_1 = 0;
      if (std::abs(boid_[i].px - boid_[j].px) < boid_[1].ds) {
        double const s = 0.5;
        vx_1 = -s * (boid_[j].px - boid_[i].px);
      }
      return vx_1;
      double vy_1 = 0;
      if (std::abs(boid_[i].py - boid_[j].py) < boid_[1].ds) {
        double const s = 0.5;
        vx_1 = -s * (boid_[j].py - boid_[i].py);
      }
      return vy_1;
    }
  }
  // Regola 2: Allineamento
  // Regola 3 : Coesione
};

#endif