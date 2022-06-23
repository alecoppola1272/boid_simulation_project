#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "flock.hpp"
#include "velocity_rules.hpp"

auto update_velocity(Flock& flock, values const& val) {
  std::vector<coordinates>::iterator it_last = std::prev(flock.end());
  for (std::vector<coordinates>::iterator it = flock.begin(); it != it_last; ++it) {
    velocity v_sum{0., 0.};
    v_sum = velocity_sum(v_sum, flock, it, val);
    v_sum = velocity_limit(v_sum, val);

    it->v.x = v_sum.x;
    it->v.y = v_sum.y;
  }

  return flock;
}

auto update_position(Flock& flock, int fps) {
  std::vector<coordinates>::iterator it_last = std::prev(flock.end());

  for (std::vector<coordinates>::iterator it = flock.begin(); it != it_last; ++it) {
    it->p.x += it->v.x / fps;
    it->p.y += it->v.y / fps;
  }

  return flock;
}

auto update_flock(int fps, Flock& flock, values const& val) {
  flock = update_velocity(flock, val);
  flock = update_position(flock, fps);

  return flock;
}

auto simulation(double duration_second, int fps, values const& val) {
  values val;
  Flock flock{{}};
  position cm;
  double steps_tot = duration_second * fps;

  flock.add_boids(val);

  std::cout << "Centro di massa:";
  for (int steps = 0; steps != steps_tot; ++steps) {
    flock = update_flock(fps, flock, val);

    cm = flock.center_mass(val.n_boids);
    std::cout << "/n"
              << "step" << steps + 1 << "   |   x = " << cm.x
              << ",  y = " << cm.y;
    // SFML
  }
}

#endif