#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <cmath>
#include <stdexcept>
#include <vector>
constexpr double ds{10.};  // Check if
constexpr double s = 0.5;

struct boid {
  double vx{};
  double vy{};
  double px{};
  double py{};
  double const ds{};
};

class flock {
  std::vector<boid> flock_;
  double d_;

 public:
  flock(std::vector<boid> flock, double d) : flock_{flock}, d_{d} {}

  int size() const { return flock_.size(); }
  boid boid_state(int i) { return flock_[i]; }

  // Struct that represent the mass center
  struct CM {
    double x_cm{};
    double y_cm{};
  };
  CM mass_center() {
    double x_tot = 0;
    double y_tot = 0;
    CM cm;
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
  //  Rule 1 : Separation
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
      if (std::abs(it->py - it_next->py) < ds) {
        sum_vy1 += std::abs(it->vy - it_next->vy);
      }
      if (std::abs(it->px - it_next->px) < ds) {
        sum_vx1 += std::abs(it->vx - it_next->vx);
      }
    }

    v1.vx_1 = -s * sum_vx1;
    v1.vy_1 = -s * sum_vy1;

    return v1;
  }
  // Rule 2: Aligment
  double aligment() {}
  // Rule 3 : Coesion
  double coesion() {}
};

#endif

// Check return multiple values (struct, array)
// Add boids ? (push_back)