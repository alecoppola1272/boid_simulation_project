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
auto alignment_velocity(int n_boids, auto it, Flock flock) {
  coordinates v2;
  coordinates v2_sum{0., 0.};

  auto it_last = std::prev(flock.fend());
  for (auto it = flock.fbegin(); it != it_last; ++it) {
    for (int i = 0; i != spatial_dimension; ++i) {
      v2.velocity[i] += it->position[i];
    }
  }

  for (int i = 0; i != spatial_dimension; i++) {
    v2.velocity[i] = alignment_factor * (v2_sum.velocity[i] - it->velocity[i]) *
                     (n_boids - 1);
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

// update velocity
auto update_velocity(Flock flock, auto it) {
  coordinates v_sum;
  auto v1 = serparation_velocity(flock);
  auto v2 = alignment_velocity();
  auto v3 = coesion_velocity();
  auto v4 = edge_velocity();

  v_sum.velocity = {0., 0.};
  v_sum.velocity += v1 + v2 + v3 + v4 + it->velocity;

  // velocity limit
  for (int i = 0; i != spatial_dimension; i++) {
    if ((std::abs(v_sum.velocity[i]) > velocity_max)) {
      if (v_sum.velocity[i] > 0) {
        v_sum.velocity[i] = velocity_max;
      } else {
        v_sum.velocity[i] = -velocity_max;
      }
    }
  }

  return v_sum.velocity;
}