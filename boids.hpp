#include <array>
#include <cmath>
#include <numeric>
#include <vector>

constexpr int spatial_dimension{2};
constexpr double distance_separation{5.};
double separation_factor;
double alignment_factor;
double coesion_factor;

struct coordinates {
  std::array<double, spatial_dimension> position;
  std::array<double, spatial_dimension> velocity;
};

class Boid {
  coordinates boid;
  double width;
  //inizializzare width

 public:
  Boid(coordinates boid_, double width_) : boid{boid_}, width{width_} {}

  // velocity rule
  // rule 1: Separation
  // auto serparation_velocity() {
  //   coordinates v1;
  //   coordinates v1_sum{0., 0.};

  //   auto it_last = std::prev(Flock::Flock.end());
  //   for (auto i = Flock::Flock.begin(); i != it_last; ++i) {
  //     for (auto j = Flock::Flock.begin(); j != it_last; ++j) {
  //       for (int k = 0; k != spatial_dimension; ++k) {
  //         if (std::abs(j->position - i->position) < distance_separation) {
  //           v1_sum.velocity += std::abs(j->velocity - i->velocity);
  //         }
  //       }
  //     }
  //   }

  //   for (int i = 0; i != spatial_dimension; ++i) {
  //     v1.velocity[i] = -separation_factor * v1_sum.velocity[i];
  //   }

  //   return v1;
  // }

  // // rule 2: Alignment
  // auto alignment_velocity(int n_boids, int* j) {
  //   coordinates v2;
  //   coordinates v2_sum{0., 0.};

  //   for (auto it = Flock::Flock.begin(); it != Flock::Flock.end(); ++it) {
  //     v2.velocity += it->position;
  //   }

  //   for (int i = 0; i != spatial_dimension; i++) {
  //     v2.velocity[i] =
  //         alignment_factor * (v2_sum.velocity[i] - j->velocity) * (n_boids - 1);
  //   }

  //   return v2;
  // }

  // // rule 3: Coesion
  // auto coesion_velocity(int n_boids) {
  //   coordinates v3;
  //   auto cm = Flock::center_mass(n_boids);
  //   auto it = Flock::Flock.begin();

  //   for (int i = 0; i != spatial_dimension; i++) {
  //     v3.velocity[i] = coesion_factor * (cm.position[i] - it->position);
  //     // prendere flock[it].position
  //   }

  //   return v3;
  // }

  // behavior edges
  auto edge_velocity() {
    coordinates edge;

    for (int i = 0; i != spatial_dimension; i++) {
      if ((boid.position[i] < 10 && boid.velocity[i] < 10) ||
          (boid.position[i] > 90 && boid.velocity[i] > -10)) {
        edge.velocity[i] -= boid.velocity[i] * (10 - boid.position[i]) * 0.1;
      }
    }

    return edge.velocity;
  }
};

class Flock {
  std::vector<Boid> flock;
  // da vettore ad array?

 public:
  Flock(std::vector<Boid> flock_) : flock{flock_} {}

  auto center_mass(int n_boids) {
    coordinates cm;
    coordinates sum;
    sum.position = {0., 0.};

    for (auto i = flock.begin(); i != flock.end(); i++) {
      std::accumulate(sum.position, flock[i].position, 0);
      // prendere flock[i].position
    }

    for (int i = 0; i != spatial_dimension; i++) {
      cm.position[i] = sum.position[i] / (n_boids - 1);
    }

    return cm.position;
  }
};
