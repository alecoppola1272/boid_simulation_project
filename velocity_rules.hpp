#ifndef VELOCITY_RULES_HPP
#define VELOCITY_RULES_HPP

// rule 1: Separation
auto serparation_velocity(Flock flock, values value) {
  coordinates v1;
  coordinates v1_sum{0., 0.};

  auto it_last = std::prev(flock.fend());
  for (auto it1 = flock.fbegin(); it1 != it_last; ++it1) {
    for (auto it2 = flock.fbegin(); it2 != it_last; ++it2) {
      if (std::abs(it2->px - it1->px) < distance_separation) {
        v1_sum.vx += std::abs(it2->vx - it1->vx);
      }
      if (std::abs(it2->py - it1->py) < distance_separation) {
        v1_sum.vy += std::abs(it2->vy - it1->vy);
      }
    }
  }

  v1.vx = -value.separation_factor * v1_sum.vx;
  v1.vy = -value.separation_factor * v1_sum.vy;

  return v1;
}

// rule 2: Alignment
auto alignment_velocity(int n_boids, auto it, Flock flock, values value) {
  coordinates v2;
  coordinates v2_sum{0., 0.};

  auto it_last = std::prev(flock.fend());
  for (auto it = flock.fbegin(); it != it_last; ++it) {
    v2.vx += it->px;
    v2.vx += it->px;
  }

  v2.vx = value.alignment_factor * (v2_sum.vx - it->vx) * (n_boids - 1);
  v2.vy = value.alignment_factor * (v2_sum.vy - it->vy) * (n_boids - 1);

  return v2;
}

// rule 3: Coesion
auto coesion_velocity(int n_boids, Flock flock, values value) {
  coordinates v3;
  auto cm = flock.center_mass(n_boids);
  auto it = flock.fbegin();

  v3.vx = value.coesion_factor * (cm.px - it->px);
  v3.vy = value.coesion_factor * (cm.py - it->py);
  // prendere flock[it].position

  return v3;
}

// behavior edges
auto edge_velocity() {
  coordinates edge;

  if ((flock.px < edge_lenght && flock.vx < velocity_default) ||
      (flock.px > (box_length - edge_lenght) && flock.vx > -velocity_default)) {
    edge.vx -= flock.vx * (edge_lenght - flock.px) * edge_factor;
  } else {
    edge.vx = 0;
  }
  if ((flock.py < edge_lenght && flock.vy < velocity_default) ||
      (flock.py > (box_length - edge_lenght) && flock.vy > -velocity_default)) {
    edge.vy -= flock.vy * (edge_lenght - flock.py) * edge_factor;
  } else {
    edge.vy = 0;
  }

  return edge;
}

// update velocity
auto update_velocity(Flock flock, auto it, values value, int n_boids) {
  coordinates v_sum;
  auto v1 = serparation_velocity(flock, value);
  auto v2 = alignment_velocity(n_boids, it, flock, value);
  auto v3 = coesion_velocity(n_boids, flock, value);
  auto v4 = edge_velocity();

  v_sum.vx = 0;
  v_sum.vx += v1.vx + v2.vx + v3.vx + v4.vx + it->vx;

  v_sum.vy = 0;
  v_sum.vy += v1.vy + v2.vy + v3.vy + v4.vy + it->vy;

  // velocity limit
  if ((std::abs(v_sum.vx) > velocity_max)) {
    if (v_sum.vx > 0) {
      v_sum.vx = velocity_max;
    } else {
      v_sum.vx = -velocity_max;
    }
  }
  if ((std::abs(v_sum.vy) > velocity_max)) {
    if (v_sum.vy > 0) {
      v_sum.vy = velocity_max;
    } else {
      v_sum.vy = -velocity_max;
    }
  }

  return v_sum;
}

#endif