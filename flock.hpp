#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <cmath>
#include <iostream>
#include <vector>

constexpr double distance_s{10.};
double separation_factor;
double alignment_factor;
double coesion_factor;

struct boid {
  double vx{};
  double vy{};
  double px{};
  double py{};
};

class Flock {
  std::vector<boid> flock_;
  double d_;

 public:
  Flock(std::vector<boid> flock, double d) : flock_{flock}, d_{d} {}

  int size() const { return flock_.size(); }
  boid boid_state(int i) { return flock_[i]; }

  // Struct for mass center
  struct CM {
    double x_cm{};
    double y_cm{};
  };

  CM mass_center() {
    CM cm;
    double x_tot = 0;
    double y_tot = 0;
    auto l = size();

    for (auto it = flock_.begin(); it != flock_.end(); ++it) {
      x_tot += it->px;
      y_tot += it->py;
    }

    cm.x_cm = x_tot / (l - 1);
    cm.y_cm = y_tot / (l - 1);
    return cm;
  }

  // Rules

  // Rule 1 : Separation
  // struct that represent separation velocity
  struct separation_v {
    double vx_1;
    double vy_1;
  };

  separation_v separation() {
    separation_v v1;
    double sum_vx1 = 0;
    double sum_vy1 = 0;

    auto it = flock_.begin();
    auto it_next = std::next(it);
    auto it_last = std::prev(flock_.end());

    //  ! Add external "for"
    for (; it != it_last; ++it, ++it_next) {
      if (std::abs(it->py - it_next->py) < distance_s) {
        sum_vy1 += std::abs(it->vy - it_next->vy);
      }
      if (std::abs(it->px - it_next->px) < distance_s) {
        sum_vx1 += std::abs(it->vx - it_next->vx);
      }
    }

    v1.vx_1 = -separation_factor * sum_vx1;
    v1.vy_1 = -separation_factor * sum_vy1;

    return v1;
  }

  // Rule 2: Alignment
  struct alignment_v {
    double vx_2;
    double vy_2;
  };

  alignment_v alignment() {
    alignment_v v2;
    double sum_vx2 = 0;
    double sum_vy2 = 0;
    auto l = size();
    int j;

    for (auto it = flock_.begin(); it != flock_.end(); ++it) {
      sum_vx2 += it->px;
      sum_vy2 += it->py;
    }

    v2.vx_2 = alignment_factor * (sum_vx2 - flock_[j].px) * (l - 1);
    v2.vy_2 = alignment_factor * (sum_vy2 - flock_[j].py) * (l - 1);

    return v2;
  }

  // Rule 3 : Coesion
  struct coesion_v {
    double vx_3;
    double vy_3;
  };

  coesion_v coesion() {
    coesion_v v3;
    CM p_cm = mass_center();
    int j;

    v3.vx_3 = coesion_factor * (p_cm.x_cm - flock_[j].px);
    v3.vy_3 = coesion_factor * (p_cm.y_cm - flock_[j].py);

    return v3;
  }
};

#endif

// Check return multiple values (struct, array)
// Variabili boid in due array: v(x,y) e p(x,y)