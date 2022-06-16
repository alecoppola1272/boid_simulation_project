#ifndef VELOCITY_RULES_HPP
#define VELOCITY_RULES_HPP

auto serparation_velocity(Flock flock, double separation_factor) {
  velocity v1;
  velocity v1_sum{0., 0.};

  auto it_last = std::prev(flock.end());
  for (auto it1 = flock.begin(); it1 != it_last; ++it1) {
    for (auto it2 = flock.begin(); it2 != it_last; ++it2) {
      if (it1 != it2 && std::abs(it1->p.x - it2->p.x) < distance_separation &&
          std::abs(it1->p.y - it2->p.y) < distance_separation) {
        v1_sum.x += std::abs(it2->v.x - it1->v.x);
        v1_sum.y += std::abs(it2->v.y - it1->v.y);
      }
    }
  }

  v1.x = -separation_factor * v1_sum.x;
  v1.y = -separation_factor * v1_sum.y;

  return v1;
}

auto alignment_velocity(int n_boids, auto it, Flock flock,
                        double alignment_factor) {
  velocity v2;
  velocity v2_sum{0., 0.};

  auto it_last = std::prev(flock.end());
  for (auto it = flock.begin(); it != it_last; ++it) {
    v2_sum.x += it->p.x;
    v2_sum.y += it->p.y;
  }

  v2.x = alignment_factor * (v2_sum.x - it->v.x) / (n_boids - 1);
  v2.y = alignment_factor * (v2_sum.y - it->v.y) / (n_boids - 1);

  return v2;
}

auto coesion_velocity(int n_boids, Flock flock, auto it,
                      double coesion_factor) {
  velocity v3;
  auto cm = flock.center_mass(n_boids);

  v3.x = coesion_factor * (cm.x - it->p.x);
  v3.y = coesion_factor * (cm.y - it->p.y);

  return v3;
}

auto edge_velocity(Flock flock, auto it) {
  velocity edge;

  auto it = flock.begin();
  auto it_last = std::prev(flock.end());

  if ((it->p.x < edge_lenght && it->v.x < velocity_default) ||
      (it->p.x > (box_length - edge_lenght) && it->v.x > -velocity_default)) {
    edge.x -= it->v.x * (edge_lenght - it->p.x) * edge_factor;
  } else {
    edge.x = 0;
  }
  if ((it->p.y < edge_lenght && it->v.y < velocity_default) ||
      (it->p.y > (box_length - edge_lenght) && it->v.y > -velocity_default)) {
    edge.y -= it->v.y * (edge_lenght - it->p.y) * edge_factor;
  } else {
    edge.y = 0;
  }

  return edge;
}

auto velocity_limit(velocity v_sum) {
  if ((std::abs(v_sum.x) > velocity_max)) {
    if (v_sum.x > 0) {
      v_sum.x = velocity_max;
    } else {
      v_sum.x = -velocity_max;
    }
  }

  if ((std::abs(v_sum.y) > velocity_max)) {
    if (v_sum.y > 0) {
      v_sum.y = velocity_max;
    } else {
      v_sum.y = -velocity_max;
    }
  }

  return v_sum;
}

auto velocity_sum(velocity v_sum, Flock flock, auto it, values val, int n_boids,
                  auto controllo_vicini) {
  auto v1 = serparation_velocity(flock, val.separation_factor);
  auto v2 = alignment_velocity(n_boids, it, flock, val.alignment_factor);
  auto v3 = coesion_velocity(n_boids, flock, it, val.coesion_factor);
  auto v4 = edge_velocity(flock, it);

  v_sum.x += v1.x + v2.x + v3.x + v4.x + it->v.x;
  v_sum.y += v1.y + v2.y + v3.y + v4.y + it->v.y;

  return v_sum;
}

#endif