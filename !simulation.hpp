#ifndef MAIN_CPP
#define MAIN_CPP

#include "!flock.hpp"

// velocity rule
// rule 1: Separation
auto serparation_velocity(Flock flock) {
  coordinates v1;
  coordinates v1_sum{0., 0.};

  auto it_last = std::prev(flock.fend());
  for (auto it1 = flock.fbegin(); it1 != it_last; ++it1) {
    for (auto it2 = flock.fbegin(); it2 != it_last; ++it2) {
      for (int i = 0; i != spatial_dimension; ++i) {
        if (std::abs(it2->position[i] - it1->position[i]) <
            distance_separation) {
          v1_sum.velocity[i] += std::abs(it2->velocity[i] - it1->velocity[i]);
        }
      }
    }
  }

  for (int i = 0; i != spatial_dimension; ++i) {
    v1.velocity[i] = -separation_factor * v1_sum.velocity[i];
  }

  return v1;
}

// rule 2: Alignment
auto alignment_velocity(int n_boids, int* j, Flock flock) {
  coordinates v2;
  coordinates v2_sum{0., 0.};

  auto it_last = std::prev(flock.fend());
  for (auto it = flock.fbegin(); it != it_last; ++it) {
    for (int i = 0; i != spatial_dimension; ++i) {
      v2.velocity[i] += it->position[i];
    }
  }

  for (int i = 0; i != spatial_dimension; i++) {
    v2.velocity[i] =
        alignment_factor * (v2_sum.velocity[i] - j->velocity[i]) * (n_boids - 1);
  }

  return v2;
}

// rule 3: Coesion
auto coesion_velocity(int n_boids, Flock flock) {
  coordinates v3;
  auto cm = flock.center_mass(n_boids);
  auto it = flock.fbegin();

  for (int i = 0; i != spatial_dimension; i++) {
    v3.velocity[i] = coesion_factor * (cm.position[i] - it->position);
    // prendere flock[it].position
  }

  return v3;
}

// behavior edges
auto edge_velocity() {
  coordinates edge;

  for (int i = 0; i != spatial_dimension; i++) {
    if ((flock.position[i] < edge_lenght &&
         flock.velocity[i] < velocity_default) ||
        (flock.position[i] > (box_length - edge_lenght) &&
         flock.velocity[i] > -velocity_default)) {
      edge.velocity[i] -=
          flock.velocity[i] * (edge_lenght - flock.position[i]) * edge_factor;
    } else {
      edge.velocity[i] = 0;
    }
  }

  return edge.velocity;
}

// aggregatore di velocità
auto boid_velocity(Flock flock) {
  auto v1 = serparation_velocity(flock);
  auto v2 = alignment_velocity(/* aggiungere elementi */);
  auto v3 = coesion_velocity(/* aggiungere elementi */);
  auto v4 = edge_velocity();

  flock.velocity += v1 + v2 + v3 + v4;

  // velocity limit
  for (int i = 0; i != spatial_dimension; i++) {
    if ((std::abs(flock.velocity) > velocity_max)) {
      if (flock.velocity > 0) {
        flock.velocity = velocity_max;
      } else {
        flock.velocity = -velocity_max;
      }
    }
  }

  return flock.velocity;
}

auto evolve_flock(int fps, int n_boids) {
  Flock flock{{}};

  flock.add_boids(n_boids);
  std::vector<int> controllo_vicini;
  auto it_last = std::prev(flock.fend());

  // controllo dei vicini
  for (auto it1 = flock.fbegin(); it1 != it_last; ++it1) {
    for (auto it2 = flock.fbegin(); it2 != it_last; ++it2) {
      if (it1 != it2) {
        for (int i = 0; i != spatial_dimension; ++i) {
          if (std::abs(it1->position[i] - it2->position[i]) <
              separation_factor) {  // segnare it2 vicino it1
            controllo_vicini.push_back(it2);
          }
        }
      }
    }
    // boid_velocity();
  }

  for (auto it = flock.fbegin(); it != it_last; ++it) {
    for (int i = 0; i != spatial_dimension; ++i) {
      it->position[i] += it->velocity[i] / fps;
    }
  }

  return flock;
}

auto simulation(double duration, int fps, int n_boids) {
  double steps_tot = duration * fps;
  for (int steps = 0; steps != steps_tot; ++steps) {
    evolve_flock(fps, n_boids);
  }
}

#endif