#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

struct boid {
  double vx{};
  double vy{};
  double px{};
  double py{};
  double const ds{};  // represent the "dimension" (radius) of each boid
  // double Getpx() const { return px; }
  // double Getpy() const { return py; }
};
// boid operator+(boid const& a, boid const& b) {return boid{a.px + b.px, a.py + b.py};}

class flock {
  std::vector<boid> flock_;
  double d_;  // rule distance

 public:
  flock(std::vector<boid> flock, double d) : flock_{flock}, d_{d} {}

  int size() const { return flock_.size(); }
  boid boid_state(int i) { return flock_[i]; }

  double mass_center() {
    double x_tot = 0;
    double y_tot = 0;
    auto l = size();
    for (int i = 0; i < l; ++i) {
      x_tot = x_tot + flock_[i].px;
      y_tot = y_tot + flock_[i].py;
    }

    // auto x_tot = std::accumulate(flock_.begin(), flock_.end(), 0, [](int sum,
    // const boid& f) { return sum + f.Getpx(); });
    // auto y_tot = std::accumulate(flock_.begin(), flock_.end(), 0, [](int sum,
    // const boid& f) { return sum + f.Getpy(); });

    auto x_cm = x_tot / (l - 1);
    auto y_cm = y_tot / (l - 1);
    return x_cm;
    return y_cm;
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