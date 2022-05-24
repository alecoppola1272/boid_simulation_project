#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <cmath>
#include <stdexcept>
#include <vector>

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
  double separation() {
    double const s = 0.5;
    for (int i, j; i != j; ++i, ++j) {  // "!check element position in vector!"
      double vx_1 = 0;
      if (std::abs(flock_[i].px - flock_[j].px) < flock_[1].ds) {
        vx_1 = -s * (flock_[j].px - flock_[i].px);
      }
      return vx_1;
      double vy_1 = 0;
      if (std::abs(flock_[i].py - flock_[j].py) < flock_[1].ds) {
        vx_1 = -s * (flock_[j].py - flock_[i].py);
      }
      return vy_1;
    }
  }
  // Rule 2: Aligment
  double aligment() {}
  // Rule 3 : Coesion
  double coesion() {}
};

#endif

// Check return multiple values (std::pair, std::tuple, struct, array)
// Add boids (push_back)