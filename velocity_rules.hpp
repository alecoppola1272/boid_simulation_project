#ifndef VELOCITY_RULES_HPP
#define VELOCITY_RULES_HPP

auto serparation_velocity(Flock& flock, values const& val) {
  velocity v1;
  velocity v1_sum{0., 0.};

  // overlap
  for (auto it1 = flock.begin(); it1 != std::prev(std::prev(flock.end()));
       ++it1) {
    for (auto it2 = std::next(it1); it2 != std::prev(flock.end()); ++it2) {
      if (std::sqrt(std::pow(it1->p.x - it2->p.x, 2.) +
                    (std::pow(it1->p.y - it2->p.y, 2.))) <=
          val.distance_separation) {
        v1_sum.x += it2->p.x - it1->p.x;
        v1_sum.y += it2->p.y - it1->p.y;
      }
    }
  }

  v1.x = -val.separation_factor * v1_sum.x;
  v1.y = -val.separation_factor * v1_sum.y;

  return v1;
}

auto alignment_velocity(Flock& flock, std::vector<coordinates>::iterator it,
                        values const& val) {
  velocity v2;
  velocity v2_sum{0., 0.};

  for (auto it1 = flock.begin(); it1 != std::prev(flock.end()); ++it1) {
    if (it1 != it) {
      v2_sum.x += it1->v.x;
      v2_sum.y += it1->v.y;
    }
  }

  v2.x = val.alignment_factor * (v2_sum.x - it->v.x) / (val.n_boids - 1);
  v2.y = val.alignment_factor * (v2_sum.y - it->v.y) / (val.n_boids - 1);

  return v2;
}

auto coesion_velocity(Flock& flock, std::vector<coordinates>::iterator it,
                      values const& val) {
  velocity v3;
  auto cm = flock.center_mass(val.n_boids);

  v3.x = val.coesion_factor * (cm.x - it->p.x);
  v3.y = val.coesion_factor * (cm.y - it->p.y);

  return v3;
}

auto edge_velocity(std::vector<coordinates>::iterator it, values const& val) {
  velocity edge;

  if ((it->p.x < val.edge_lenght && it->v.x < val.velocity_default) ||
      (it->p.x > (val.box_length - val.edge_lenght) &&
       it->v.x > -val.velocity_default)) {
    edge.x -= it->v.x * (val.edge_lenght - it->p.x) * val.edge_factor;
  } else {
    edge.x = 0;
  }
  if ((it->p.y < val.edge_lenght && it->v.y < val.velocity_default) ||
      (it->p.y > (val.box_length - val.edge_lenght) &&
       it->v.y > -val.velocity_default)) {
    edge.y -= it->v.y * (val.edge_lenght - it->p.y) * val.edge_factor;
  } else {
    edge.y = 0;
  }

  return edge;
}

auto velocity_limit(velocity v_sum, values const& val) {
  if ((std::abs(v_sum.x) > val.velocity_max)) {
    if (v_sum.x > 0) {
      v_sum.x = val.velocity_max;
    } else {
      v_sum.x = -val.velocity_max;
    }
  }

  if ((std::abs(v_sum.y) > val.velocity_max)) {
    if (v_sum.y > 0) {
      v_sum.y = val.velocity_max;
    } else {
      v_sum.y = -val.velocity_max;
    }
  }

  return v_sum;
}

auto velocity_sum(velocity v_sum, Flock& flock,
                  std::vector<coordinates>::iterator it, values const& val) {
  auto v1 = serparation_velocity(flock, val);
  auto v2 = alignment_velocity(flock, it, val);
  auto v3 = coesion_velocity(flock, it, val);
  auto v4 = edge_velocity(it, val);

  v_sum.x += v1.x + v2.x + v3.x + v4.x + it->v.x;
  v_sum.y += v1.y + v2.y + v3.y + v4.y + it->v.y;

  return v_sum;
}

#endif