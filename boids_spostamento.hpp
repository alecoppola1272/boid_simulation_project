#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <cmath>
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

class Boid {
  std::vector<boid> boid_;
  double d_;

 public:
  Boid(std::vector<boid> boid, double d) : boid_{boid}, d_{d} {}

  int size() const { return boid_.size(); }
  boid boid_state(int i) { return boid_[i]; }

  // Rule 1 : Separation
  struct separation_v {
    double vx_1;
    double vy_1;
  };

  separation_v separation() {
    separation_v v1;
    double sum_vx1{};
    double sum_vy1{};

    auto it_last = std::prev(boid_.end());

    for (auto it2 = boid_.begin(); it2 != it_last; ++it2) {
      for (auto it1 = boid_.begin(); it1 != it_last; ++it1) {
        if (std::abs(it1->py - it2->py) < distance_s) {
          sum_vy1 += std::abs(it1->vy - it2->vy);
        }
        if (std::abs(it1->px - it2->px) < distance_s) {
          sum_vx1 += std::abs(it1->vx - it2->vx);
        }
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

  alignment_v alignment(int size, int* j) {
    alignment_v v2;
    double sum_vx2{};
    double sum_vy2{};

    auto it = boid_.begin();
    for (; it != boid_.end(); ++it) {
      sum_vx2 += it->px;
      sum_vy2 += it->py;
    }

    v2.vx_2 = alignment_factor * (sum_vx2 - j->vx) * (size - 1);
    v2.vy_2 = alignment_factor * (sum_vy2 - j->vy) * (size - 1);

    return v2;
  }

  // Rule 3 : Coesion
  struct coesion_v {
    double vx_3;
    double vy_3;
  };

  coesion_v coesion(int j, int size) {
    coesion_v v3;
    CM cm = mass_center(size);
    auto it = boid_.begin();

    v3.vx_3 = coesion_factor * (cm.x_cm - it->px);
    v3.vy_3 = coesion_factor * (cm.y_cm - it->py);

    return v3;
  }

  // Behavior edges
  struct border_v {
    double vx_b;
    double vy_b;
  };

  border_v borders() {
    border_v vb;
    auto it = boid_.begin();
    auto it_next = std::next(it);
    auto it_last = std::prev(boid_.end());

    for (; it != it_last; ++it, ++it_next) {
      if ((it->px < 10 && it->vx < 0) || (it->px > 90 && it->vx > 0)) {
        it->vx -= it->vx * (10 - it->px) * 0, 1;
      }
      if ((it->py < 10 && it->vy < 0) || (it->py > 90 && it->vy > 0)) {
        it->vy -= it->vy * (10 - it->py) * 0, 1;
      }
    }

    return vb;
  }
};

#endif

// Check return multiple values (struct, array)
// Variabili boid in due array: v(x,y) e p(x,y)