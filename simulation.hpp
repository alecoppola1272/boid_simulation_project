#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "flock.hpp"
#include "velocity_rules.hpp"
#include <iostream>

auto update_velocity(Flock& flock, values const& val) {
  std::vector<coordinates>::iterator it_last = std::prev(flock.end());
  for (std::vector<coordinates>::iterator it = flock.begin(); it != it_last;
       ++it) {
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

  for (std::vector<coordinates>::iterator it = flock.begin(); it != it_last;
       ++it) {
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

void simulation(values const& val, double duration_second, int fps) {
  Flock flock{{}};
  position cm{};
  velocity vm{};
  double steps_tot = duration_second * fps;

  flock.add_boids(val);

  std::cout << "Velocità media:\nStep\t|  vx\t\t|  vy\t\t|  px\t\t|  py\n-------------------------------------------------------------------";
  for (int steps = 0; steps != steps_tot; ++steps) {
    flock = update_flock(fps, flock, val);

    cm = flock.center_mass(val.n_boids);
    vm = flock.velocity_mean(val.n_boids);
    if ((steps + 1) % 10 == 0) {
      std::cout << '\n'
                << steps + 1 << "\t|  " << vm.x << "\t|  " << vm.y << "\t|  "
                << cm.x << "\t|  " << cm.y;
      // SFML
      // cin e cout nel main
    }
  }
  std::cout << "\n\n";
}

#endif