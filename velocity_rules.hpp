#ifndef VELOCITY_RULES_HPP
#define VELOCITY_RULES_HPP

inline coordinates separation_velocity(
    std::vector<std::vector<boid>::iterator> const& neighbors, boid b1,
    values const& val) {
  coordinates init{};
  coordinates p_sum =
      std::accumulate(neighbors.begin(), neighbors.end(), init,
                      [=](coordinates const a, auto const b) {
                        coordinates result{};
                        if (std::hypot(b1.p.x - b->p.x, b1.p.y - b->p.y) <=
                            val.distance_separation) {
                          result = a + b->p - b1.p;
                        }
                        return result;
                      });
  return p_sum * -val.separation_factor;
}

inline coordinates alignment_velocity(
    std::vector<std::vector<boid>::iterator> const& neighbors, boid b1,
    values const& val) {
  coordinates init{};
  coordinates v2_sum = std::accumulate(neighbors.begin(), neighbors.end(), init,
                                       [](coordinates const a, auto b) {
                                         return a + b->v;
                                       });
  return (v2_sum - b1.v) * val.alignment_factor / (val.n_boids - 1);
}

inline coordinates center_mass(
    std::vector<std::vector<boid>::iterator> const& neighbors) {
  coordinates init{};
  coordinates p_sum = std::accumulate(
      neighbors.begin(), neighbors.end(), init,
      [=](coordinates const a, auto const b) { return a + b->p; });
  return p_sum / neighbors.size();
}

inline coordinates coesion_velocity(
    std::vector<std::vector<boid>::iterator> const& neighbors, boid b1,
    values const& val) {
  return (center_mass(neighbors) - b1.p) * val.coesion_factor;
}

inline coordinates velocity_edge(boid b1, values const& val) {
  coordinates v_edge{};
  if (b1.p.x < val.edge_lenght && b1.v.x < val.velocity_default) {  // left
    (b1.v.x < 0)  // ? outwards : inwards
        ? (v_edge.x -= b1.v.x * (val.edge_lenght - b1.p.x) * val.edge_factor)
        : (v_edge.x += b1.v.x * (val.edge_lenght - b1.p.x) * val.edge_factor);
  }
  if (b1.p.y < val.edge_lenght && b1.v.y < val.velocity_default) {  // down
    (b1.v.y < 0)
        ? (v_edge.y -= b1.v.y * (val.edge_lenght - b1.p.y) * val.edge_factor)
        : (v_edge.y += b1.v.y * (val.edge_lenght - b1.p.y) * val.edge_factor);
  }
  if (b1.p.x > (val.box_length - val.edge_lenght) &&
      b1.v.x > -val.velocity_default) {  // right
    (b1.v.x > 0)
        ? (v_edge.x += b1.v.x * ((val.box_length - val.edge_lenght) - b1.p.x) *
                       val.edge_factor)
        : (v_edge.x -= b1.v.x * ((val.box_length - val.edge_lenght) - b1.p.x) *
                       val.edge_factor);
  }
  if (b1.p.y > (val.box_length - val.edge_lenght) &&
      b1.v.y > -val.velocity_default) {  // up
    (b1.v.y > 0)
        ? (v_edge.y += b1.v.y * ((val.box_length - val.edge_lenght) - b1.p.y) *
                       val.edge_factor)
        : (v_edge.y -= b1.v.y * ((val.box_length - val.edge_lenght) - b1.p.y) *
                       val.edge_factor);
  }
  return v_edge;
}

inline coordinates velocity_maximum(coordinates v, values const& val) {
  if (v.x > std::sqrt(pow(val.velocity_max,2)/2)) {
    v.x = std::sqrt(pow(val.velocity_max,2)/2);
  }
  if (v.x < -std::sqrt(pow(val.velocity_max,2)/2)) {
    v.x = -std::sqrt(pow(val.velocity_max,2)/2);
  }
  if (v.y > std::sqrt(pow(val.velocity_max,2)/2)) {
    v.y = std::sqrt(pow(val.velocity_max,2)/2);
  }
  if (v.y < -std::sqrt(pow(val.velocity_max,2)/2)) {
    v.y = -std::sqrt(pow(val.velocity_max,2)/2);
  }
  return v;
}

inline coordinates velocity_minimum(coordinates v, values const& val) {
  if (std::abs(v.x) < val.velocity_min) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> n_rand(0, 1);
    (n_rand(gen) == 0) ? (v.x = val.velocity_min) : (v.x = -val.velocity_min);
  }
  if (std::abs(v.y) < val.velocity_min) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> n_rand(0, 1);
    (n_rand(gen) == 0) ? (v.y = val.velocity_min) : (v.y = -val.velocity_min);
  }
  return v;
}

inline coordinates velocity_limit(coordinates v, values const& val) {
  v = velocity_maximum(v, val);
  v = velocity_minimum(v, val);
  return v;
}

inline coordinates velocity_rules_sum(
    std::vector<std::vector<boid>::iterator> const& neighbors, boid b1,
    values const& val) {
  return separation_velocity(neighbors, b1, val) +
         alignment_velocity(neighbors, b1, val) +
         coesion_velocity(neighbors, b1, val);
}

#endif