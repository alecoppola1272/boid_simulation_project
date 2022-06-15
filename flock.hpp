#ifndef FLOCK_HPP
#define FLOCK_HPP

struct position {
  double x;
  double y;
};

struct velocity {
  double x;
  double y;
};

struct coordinates {
  position p;
  velocity v;
};

class Flock {
  std::vector<coordinates> flock;

 public:
  Flock(std::vector<coordinates> flock_) : flock{flock_} {}

  auto add_boids(int n_boids) {
    coordinates new_boid;

    std::uniform_real_distribution<> v_rand(1., velocity_default);
    std::uniform_real_distribution<> p_rand(1., box_length - 1);

    for (int j = 0; j != n_boids; ++j) {
      // new_boid.v.x = v_rand;
      // new_boid.v.y = v_rand;

      // new_boid.p.x = p_rand;
      // new_boid.p.y = p_rand;

      flock.push_back(new_boid);
    }
    return flock;
  }

  auto center_mass(int n_boids) {
    position cm;
    position sum;
    sum.x = 0;
    sum.y = 0;

    // sum = std::accumulate(flock.begin(), flock.end(), 
    // Position{0.,0.},[](...) { return ...; });

    cm.x = sum.x / (n_boids - 1);
    cm.y = sum.y / (n_boids - 1);

    return cm;
  }

  auto begin() { return flock.begin(); }
  auto end() { return flock.end(); }
};

#endif