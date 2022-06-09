#ifndef MAIN_CPP
#define MAIN_CPP

#include "flock.hpp"

auto evolve_flock(int fps, int n_boids, Flock flock) {
  flock.add_boids(n_boids);

  // // controllo dei vicini

  // auto it = flock.begin();
  // auto it_last = std::prev(flock.end());

  // for (; it != it_last; ++it) {
  //   for (int i = 0; i != spatial_dimension; ++i) {
  //     it->flock_evolve.position[i] += it->flock_evolve.velocity[i] / fps;
  //   }
  // }

  return flock;
}

auto simulation(double duration, int fps, int n_boids, Flock flock) {
  double steps_tot = duration * fps;
  for (int steps = 0; steps != steps_tot; ++steps) {
    evolve_flock(fps, n_boids, flock);
  }
}

#endif

// // velocity rule
// // rule 1: Separation
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

//   for (auto it = Flock::flock.begin(); it != Flock::Flock.end(); ++it) {
//     v2.velocity += it->position;
//   }

//   for (int i = 0; i != spatial_dimension; i++) {
//     v2.velocity[i] =
//         alignment_factor * (v2_sum.velocity[i] - j->velocity) * (n_boids -
//         1);
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

// // behavior edges
// auto edge_velocity() {
//   coordinates edge;

//   for (int i = 0; i != spatial_dimension; i++) {
//     if ((boid.position[i] < 10 && boid.velocity[i] < velocity_default) ||
//         (boid.position[i] > 90 && boid.velocity[i] > -velocity_default)) {
//       edge.velocity[i] -= boid.velocity[i] * (10 - boid.position[i]) * 0.1;
//     } else {
//       edge.velocity[i] = 0;
//     }
//   }

//   return edge.velocity;
// }

// auto boid_velocity() {
//   auto v1 = serparation_velocity();
//   auto v2 = alignment_velocity(/* aggiungere elementi */);
//   auto v3 = coesion_velocity(/* aggiungere elementi */);
//   auto v4 = edge_velocity();

//   boid.velocity += v1 + v2 + v3 + v4;

//   // velocity limit
//   for (int i = 0; i != spatial_dimension; i++) {
//     if ((std::abs(boid.velocity) > velocity_max)) {
//       if (boid.velocity > 0) {
//         boid.velocity = velocity_max;
//       } else {
//         boid.velocity = -velocity_max;
//       }
//     }
//   }

//   return boid.velocity;
// }