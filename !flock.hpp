#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <array>
#include <cmath>
#include <numeric>
#include <vector>

constexpr int spatial_dimension{2};
constexpr int box_length{100};
constexpr int edge_lenght{10};
constexpr double velocity_default{10.};
constexpr double velocity_max{20.};
constexpr double distance_separation{5.};
constexpr double edge_factor{0.1};
double separation_factor;
double alignment_factor;
double coesion_factor;

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
        new_boid.velocity[i] = std::rand() % 1, velocity_default;
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

    for (auto i = flock.begin(); i != flock.end(); i++) {
      std::accumulate(sum.position, i->position, 0);
    }

    for (int i = 0; i != spatial_dimension; i++) {
      cm.position[i] = sum.position[i] / (n_boids - 1);
    }

    return cm.position;
  }

  auto fbegin() { return flock.begin(); }

  auto fend() { return flock.end(); }
};

#endif