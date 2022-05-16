#ifndef BOID_HPP
#define BOID_HPP

struct boids {
  double vx{};
  double vy{};
  double px{};
  double py{};
  double ds{};  // represent the "dimension" (radius) of each boid
};

#endif