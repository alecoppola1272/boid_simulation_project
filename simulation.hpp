#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "flock.hpp"
#include "velocity_rules.hpp"

auto update_velocity(Flock& flock, values const& val) {
  for (auto it1 = flock.begin(); it1 != std::prev(flock.end()); ++it1) {
    std::vector<std::vector<coordinates>::iterator> neighbors;
    for (auto it2 = flock.begin(); it2 != std::prev(flock.end()); ++it2) {
      if (it2 != it1 && std::sqrt(std::pow(std::abs(it1->p.x - it2->p.x), 2.) +
                                  (std::pow(std::abs(it1->p.y - it2->p.y),
                                            2.))) <= val.distance_neighbors) {
        // vista boid
        neighbors.push_back(it1);  // iteratore o coordinates?
      }
    }

    velocity v_sum{0., 0.};
    v_sum = velocity_sum(v_sum, neighbors, it1, val);
    v_sum = velocity_limit(v_sum, val);

    it1->v.x = v_sum.x;
    it1->v.y = v_sum.y;
  }

  return flock;
}

auto update_position(Flock& flock, int fps) {
  auto it_last = std::prev(flock.end());

  for (auto it = flock.begin(); it != it_last; ++it) {
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
  position cm;
  velocity vm;
  position dsm;
  double steps_tot = duration_second * fps;

  flock.add_boids(val);

  std::cout << "Step\t|  vm x\t\t|  vm y\t\t|  cm x\t\t|  cm y\t\t|  dsm x\t|  "
               "dsm y\n";
  std::cout << "---------------------------------------------------------------"
               "-----------------------------------";
  for (int steps = 0; steps != steps_tot; ++steps) {
    flock = update_flock(fps, flock, val);

    cm = flock.center_mass(val.n_boids);
    vm = flock.velocity_mean(val.n_boids);
    dsm = flock.dseparation_mean(val.n_boids);
    if ((steps + 1) % val.visual_steps == 0) {
      std::cout << '\n'
                << steps + 1 << "\t|  " << vm.x << "\t|  " << vm.y << "\t|  "
                << cm.x << "\t|  " << cm.y << "\t|  " << dsm.x << "\t|  "
                << dsm.y;
      // SFML
    }
  }
  std::cout << "\n\n";
}

#endif