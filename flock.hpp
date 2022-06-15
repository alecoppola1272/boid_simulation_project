#ifndef FLOCK_HPP
#define FLOCK_HPP

struct coordinates {
  double px;
  double py;
  double vx;
  double vy;
};  // struct annidata?

class Flock {
  std::vector<coordinates> flock;

 public:
  Flock(std::vector<coordinates> flock_) : flock{flock_} {}

  auto add_boids(int n_boids) {
    coordinates new_boid;
    std::uniform_real_distribution<> v_rand;  // (1., velocity_default)
    std::uniform_real_distribution<> p_rand;  // (1., box_length - 1)

    for (int j = 0; j != n_boids; ++j) {
      new_boid.vx = v_rand();
      new_boid.vy = v_rand;

      new_boid.px = p_rand;
      new_boid.py = p_rand;

      flock.push_back(new_boid);
    }
    return flock;
  }

  auto center_mass(int n_boids) {
    coordinates cm;
    coordinates sum;
    sum.px = 0;
    sum.py = 0;

    sum.px = std::accumulate(flock.begin(), flock.end(), Position{0., 0.},
                             [](...) { return ...; });
    sum.py = std::accumulate(flock.begin(), flock.end(), Position{0., 0.},
                             [](...) { return ...; });

    cm.px = sum.px / (n_boids - 1);
    cm.py = sum.py / (n_boids - 1);

    return cm;
  }

  auto fbegin() { return flock.begin(); }

  auto fend() { return flock.end(); }
};

#endif