#ifndef SIMULATION_HPP
#define SIMULATION_HPP

struct coordinates {
  std::array<double, spatial_dimension> position;
  std::array<double, spatial_dimension> velocity;
};

class Flock {
  std::vector<coordinates> flock;

 public:
  Flock(std::vector<coordinates> flock_) : flock{flock_} {}

  auto add_boids(int n_boids) {
    coordinates new_boid;
    for (int j = 0; j != n_boids; ++j) {
      for (int i = 0; i != spatial_dimension; ++i) {
        new_boid.velocity[i] = std::rand() % 1,
        velocity_default;  // std::uniform_real_distribution
        new_boid.position[i] = std::rand() % 1, box_length - 1;
      }
      flock.push_back(new_boid);
    }
    return flock;
  }

  auto center_mass(int n_boids) {
    coordinates cm;
    coordinates sum;
    sum.position = {0., 0.};

    sum.position = std::accumulate(flock.begin(), flock.end(), Position{0.,0.}, [](...){return ...;});

    for (int i = 0; i != spatial_dimension; i++) {
      cm.position[i] = sum.position[i] / (n_boids - 1);
    }

    return cm.position;
  }

  auto fbegin() { return flock.begin(); }

  auto fend() { return flock.end(); }
};

#endif