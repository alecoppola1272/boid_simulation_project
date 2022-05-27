#include "boids_spostamento.hpp"
#include <vector>

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

  CM mass_center(int size) {
    CM cm;
    double x_tot{};
    double y_tot{};

    for (auto it = flock_.begin(); it != flock_.end(); ++it) {
      x_tot += it->px;
      y_tot += it->py;
    }

    cm.x_cm = x_tot / (size - 1);
    cm.y_cm = y_tot / (size - 1);
    return cm;
  }
};
