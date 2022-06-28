#ifndef VELOCITY_RULES_HPP
#define VELOCITY_RULES_HPP

auto serparation_velocity(
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    std::vector<coordinates>::iterator const& it1, values const& val) {
  velocity v1_sum{};
  for (auto it2 = neighbors.begin(); it2 != std::prev(neighbors.end()); ++it2) {
    std::vector<coordinates>::iterator it2_ = *it2;
    if (std::hypot(it1->p.x - it2_->p.x, it1->p.y - it2_->p.y) <=
        val.distance_separation) {
      v1_sum.x += it2_->p.x - it1->p.x;
      v1_sum.y += it2_->p.y - it1->p.y;
    }
  }

  velocity v1{};
  v1.x = -val.separation_factor * v1_sum.x;
  v1.y = -val.separation_factor * v1_sum.y;

  return v1;
}

auto alignment_velocity(
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    std::vector<coordinates>::iterator const& it, values const& val) {
  velocity v2_sum{};
  for (auto it1 = neighbors.begin(); it1 != std::prev(neighbors.end()); ++it1) {
    std::vector<coordinates>::iterator it1_ = *it1;
    if (it1_ != it) {
      v2_sum.x += it1_->v.x;
      v2_sum.y += it1_->v.y;
    }
  }

  velocity v2{};
  v2.x = val.alignment_factor * (v2_sum.x - it->v.x) / (val.n_boids - 1);
  v2.y = val.alignment_factor * (v2_sum.y - it->v.y) / (val.n_boids - 1);

  return v2;
}

auto center_mass(
    int const& n_boids,
    std::vector<std::vector<coordinates>::iterator> const& neighbors) {
  position sum{};
  for (auto it = neighbors.begin(); it != std::prev(neighbors.end()); ++it) {
    std::vector<coordinates>::iterator it_ = *it;

    sum.x += it_->p.x;
    sum.y += it_->p.y;
  };

  position cm{};
  cm.x = sum.x / (n_boids - 1);
  cm.y = sum.y / (n_boids - 1);

  return cm;
}

auto coesion_velocity(
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    std::vector<coordinates>::iterator const& it, values const& val) {
  position cm = center_mass(val.n_boids, neighbors);

  velocity v3{};
  v3.x = val.coesion_factor * (cm.x - it->p.x);
  v3.y = val.coesion_factor * (cm.y - it->p.y);

  return v3;
}

auto edge_velocity(std::vector<coordinates>::iterator const& it,
                   values const& val) {
  velocity edge{};

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

void velocity_limit(std::vector<coordinates>::iterator const& it,
                    values const& val) {
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

void velocity_sum(
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    std::vector<coordinates>::iterator const& it, values const& val) {
  auto v1 = serparation_velocity(neighbors, it, val);
  auto v2 = alignment_velocity(neighbors, it, val);
  auto v3 = coesion_velocity(neighbors, it, val);
  auto v4 = edge_velocity(it, val);

  it->v.x += v1.x + v2.x + v3.x + v4.x;
  it->v.y += v1.y + v2.y + v3.y + v4.y;
}

#endif