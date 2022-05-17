#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <stdexcept>
#include <vector>

#include "boid.hpp"

class flock {
  std::vector<boids> boid_;  // vector which represent each boid
  double d_;

 public:
  flock(std::vector<boids> boid, double d) : boid_{boid}, d_{d} {};

  int size() const { return boid_.size(); }

  // refit with pointers
  double mass_center() {
    double x_tot = 0;  // sum of x position
    double y_tot = 0;  // sum of y position
    int l = size();
    for (int i = 1; i <= l; ++i) {
      x_tot = x_tot + boid_[i].px;
      y_tot = y_tot + boid_[i].py;
    }
    auto x_cm = x_tot / (l - 1);
    auto y_cm = y_tot / (l - 1);
    return x_cm;
    return y_cm;
  }
};



#endif