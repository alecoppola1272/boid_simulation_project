#ifndef VELOCITY_RULES_HPP
#define VELOCITY_RULES_HPP

auto overlap(std::vector<std::vector<coordinates>::iterator> const& neighbors,
             values const& val, velocity& v1_sum) {
  for (auto it1 = neighbors.begin();
       it1 != std::prev(std::prev(neighbors.end())); ++it1) {
    std::vector<coordinates>::iterator it1_ = *it1;
    for (auto it2 = std::next(it1); it2 != std::prev(neighbors.end()); ++it2) {
      std::vector<coordinates>::iterator it2_ = *it2;

      if (std::sqrt(std::pow(it1_->p.x - it2_->p.x, 2.) +
                    (std::pow(it1_->p.y - it2_->p.y, 2.))) <=
          val.distance_separation) {
        v1_sum.x += it2_->p.x - it1_->p.x;
        v1_sum.y += it2_->p.y - it1_->p.y;
      }
    }
  }
  return v1_sum;
}

auto serparation_velocity(
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    values const& val) {
  velocity v1{};
  velocity v1_sum{0., 0.};

  v1_sum = overlap(neighbors, val, v1_sum);

  v1.x = -val.separation_factor * v1_sum.x;
  v1.y = -val.separation_factor * v1_sum.y;

  return v1;
}

auto alignment_velocity(
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    std::vector<coordinates>::iterator const it, values const& val) {
  velocity v2{};
  velocity v2_sum{0., 0.};

  for (auto it1 = neighbors.begin(); it1 != std::prev(neighbors.end()); ++it1) {
    std::vector<coordinates>::iterator it1_ = *it1;
    if (it1_ != it) {
      v2_sum.x += it1_->v.x;
      v2_sum.y += it1_->v.y;
    }
  }

  v2.x = val.alignment_factor * (v2_sum.x - it->v.x) / (val.n_boids - 1);
  v2.y = val.alignment_factor * (v2_sum.y - it->v.y) / (val.n_boids - 1);

  return v2;
}

auto center_mass(
    position cm, int const& n_boids,
    std::vector<std::vector<coordinates>::iterator> const& neighbors) {
  position sum{};

  for (auto it = neighbors.begin(); it != std::prev(neighbors.end()); ++it) {
    std::vector<coordinates>::iterator it_ = *it;

    sum.x += it_->p.x;
    sum.y += it_->p.y;
  };

  cm.x = sum.x / (n_boids - 1);
  cm.y = sum.y / (n_boids - 1);

  return cm;
}

auto coesion_velocity(
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    std::vector<coordinates>::iterator const it, values const& val) {
  velocity v3{};
  position cm{};

  cm = center_mass(cm, val.n_boids, neighbors);

  v3.x = val.coesion_factor * (cm.x - it->p.x);
  v3.y = val.coesion_factor * (cm.y - it->p.y);

  return v3;
}

auto edge_velocity(std::vector<coordinates>::iterator const it,
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

auto velocity_sum(
    velocity v_sum,
    std::vector<std::vector<coordinates>::iterator> const& neighbors,
    std::vector<coordinates>::iterator const it, values const& val) {
  auto v1 = serparation_velocity(neighbors, val);
  auto v2 = alignment_velocity(neighbors, it, val);
  auto v3 = coesion_velocity(neighbors, it, val);
  auto v4 = edge_velocity(it, val);

  v_sum.x += v1.x + v2.x + v3.x + v4.x + it->v.x;
  v_sum.y += v1.y + v2.y + v3.y + v4.y + it->v.y;

  return v_sum;
}

#endif