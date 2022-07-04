#ifndef VELOCITY_RULES_HPP
#define VELOCITY_RULES_HPP

auto separation_velocity(
    std::vector<std::vector<boid>::iterator> const& neighbors,
    std::vector<boid>::iterator const& it1, values const& val) {
  coordinates p_sum{};
  for (auto it2 = neighbors.begin(); it2 != neighbors.end(); ++it2) {
    std::vector<boid>::iterator it2_ = *it2;
    if (std::hypot(it1->p.x - it2_->p.x, it1->p.y - it2_->p.y) <=
        val.distance_separation) {
      p_sum = p_sum + it2_->p - it1->p;
    }
  }

  coordinates v1 = p_sum * -val.separation_factor;
  return v1;
}

auto alignment_velocity(
    std::vector<std::vector<boid>::iterator> const& neighbors,
    std::vector<boid>::iterator const& it, values const& val) {
  coordinates v2_sum{};
  for (auto it1 = neighbors.begin(); it1 != neighbors.end(); ++it1) {
    std::vector<boid>::iterator it1_ = *it1;
    if (it1_ != it) {
      v2_sum = v2_sum + it1_->v;
    }
  }

  coordinates v2 = (v2_sum - it->v) * val.alignment_factor / (val.n_boids - 1);
  return v2;
}

auto center_mass(int const& n_boids,
                 std::vector<std::vector<boid>::iterator> const& neighbors) {
  coordinates p_sum{};
  for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
    std::vector<boid>::iterator it_ = *it;
    p_sum = p_sum + it_->p;
  };

  coordinates cm = p_sum / (n_boids - 1);
  return cm;
}

auto coesion_velocity(std::vector<std::vector<boid>::iterator> const& neighbors,
                      std::vector<boid>::iterator const& it,
                      values const& val) {
  coordinates cm = center_mass(val.n_boids, neighbors);
  coordinates v3 = (cm - it->p) * val.coesion_factor;
  return v3;
}

auto velocity_edge(std::vector<boid>::iterator const& it, values const& val) {
  coordinates v_edge{};
  if ((it->p.x < val.edge_lenght && it->v.x < val.velocity_default) ||
      (it->p.x > (val.box_length - val.edge_lenght) &&
       it->v.x > -val.velocity_default)) {
    v_edge.x -= it->v.x * (val.edge_lenght - it->p.x) * val.edge_factor;
  } else {
    v_edge.x = 0;
  }
  if ((it->p.y < val.edge_lenght && it->v.y < val.velocity_default) ||
      (it->p.y > (val.box_length - val.edge_lenght) &&
       it->v.y > -val.velocity_default)) {
    v_edge.y -= it->v.y * (val.edge_lenght - it->p.y) * val.edge_factor;
  } else {
    v_edge.y = 0;
  }

  return v_edge;
}

void velocity_limit(std::vector<boid>::iterator const& it, values const& val) {
  if ((std::abs(it->v.x) > val.velocity_max)) {
    if (it->v.x > 0) {
      it->v.x = val.velocity_max;
    } else {
      it->v.x = -val.velocity_max;
    }
  }

  if ((std::abs(it->v.y) > val.velocity_max)) {
    if (it->v.y > 0) {
      it->v.y = val.velocity_max;
    } else {
      it->v.y = -val.velocity_max;
    }
  }
}

void velocity_sum(std::vector<std::vector<boid>::iterator> const& neighbors,
                  std::vector<boid>::iterator const& it, values const& val) {
  it->v = it->v + separation_velocity(neighbors, it, val) +
          alignment_velocity(neighbors, it, val) +
          coesion_velocity(neighbors, it, val);
}

#endif