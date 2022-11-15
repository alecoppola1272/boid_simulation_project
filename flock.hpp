#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <numeric>
#include <random>

struct values {
  // input values
  int n_boids{};
  float separation_factor;
  float alignment_factor;
  float coesion_factor;

  // box values
  float const edge_factor{0.005};
  int const box_length{600};
  int const edge_lenght{100};

  // velocity values
  int const velocity_step_update{10};
  float const velocity_default{30.};
  float const velocity_max{60.};
  float const velocity_min{5.};
  float const velocity_balancer{1.01};

  // separation values
  float const distance_neighbors{60.};
  float const distance_separation{15.};
  float const boid_blind_angle{90.};
};
struct coordinates {
  float x;
  float y;
};

struct boid {
  coordinates p;
  coordinates v;
};

coordinates operator+(coordinates const&, coordinates const&);
coordinates operator+=(coordinates&, coordinates const&);
coordinates operator-(coordinates const&, coordinates const&);
coordinates operator*(coordinates const&, float const&);
coordinates operator*=(coordinates&, float const&);
coordinates operator/(coordinates const&, float const&);

class Flock {
  std::vector<boid> flock_;

 public:
  Flock(std::vector<boid> flock) : flock_{flock} {}
  Flock() : flock_{} {};

  void add_boids(values const&);
  void update_flock(Flock&, values const&, int const&, int const&);
  float d_separation_mean();
  float velocity_mean(int const&);
  coordinates center_mass(int const&) const;
  coordinates velocity_cm(int const&);
  std::vector<boid>::iterator begin();
  std::vector<boid>::iterator end();
  int size() const;
};

float stddev(std::vector<float> const&);

#endif