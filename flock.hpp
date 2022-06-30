#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <numeric>
#include <random>

struct coordinates {
  double x;
  double y;
};

coordinates operator+(coordinates const& a, coordinates const& b) {
  return coordinates{a.x + b.x, a.y + b.y};
}
coordinates operator-(coordinates const& a, coordinates const& b) {
  return coordinates{a.x - b.x, a.y - b.y};
}
coordinates operator*(coordinates const& a, double const& b) {
  return coordinates{a.x * b, a.y * b};
}
coordinates operator/(coordinates const& a, double const& b) {
  return coordinates{a.x / b, a.y / b};
}

struct boid {
  coordinates p;
  coordinates v;
};

class Flock {
  std::vector<boid> flock_;

 public:
  Flock(std::vector<boid> flock) : flock_{flock} {}

  void add_boids(values const& val) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> v_rand(1., val.velocity_default);
    std::uniform_real_distribution<> p_rand(1., val.box_length - 1);

    boid new_boid{};
    for (int j = 0; j != val.n_boids; ++j) {
      new_boid.v.x = v_rand(gen);
      new_boid.v.y = v_rand(gen);
      new_boid.p.x = p_rand(gen);
      new_boid.p.y = p_rand(gen);

      flock_.push_back(new_boid);
    }
  }

  auto velocity_mean(int const& n_boids) {
    coordinates v_sum{};
    for (auto it = flock_.begin(); it != std::prev(flock_.end()); ++it) {
      v_sum = v_sum + it->v;
    };

    coordinates vm = v_sum / n_boids;
    return vm;
  }

  auto d_separation_mean() {
    coordinates p_sum{};
    int i{};

    for (auto it1 = flock_.begin(); it1 != std::prev(std::prev(flock_.end()));
         ++it1) {
      for (auto it2 = std::next(it1); it2 != std::prev(flock_.end()); ++it2) {
        p_sum.x += std::abs(it1->p.x - it2->p.x);
        p_sum.y += std::abs(it1->p.y - it2->p.y);
        ++i;
      }
    }

    coordinates dsm = p_sum / i;
    return dsm;
  }

  // inutile per esame
  auto center_mass(int const& n_boids) {
    coordinates p_sum{};
    for (auto it = flock_.begin(); it != std::prev(flock_.end()); ++it) {
      p_sum = p_sum + it->p;
    };

    // coordinates start{};
    // coordinates p_sum = std::accumulate(
    //     flock_.begin(), flock_.end(), start,
    //     [](boid first, boid second) { return first.p + second.p; });
    //     [](std::vector<boid>::iterator it1, std::vector<boid>::iterator it2) {
    //       return it1->p + it2->p;
    //     });

    coordinates cm = p_sum / (n_boids - 1);
    return cm;
  }

  auto begin() { return flock_.begin(); }
  auto end() { return flock_.end(); }
};

#endif