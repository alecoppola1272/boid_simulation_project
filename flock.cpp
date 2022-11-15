#include "flock.hpp"

#include "evolve.hpp"

coordinates operator+(coordinates const& a, coordinates const& b) {
  return coordinates{a.x + b.x, a.y + b.y};
}
coordinates operator+=(coordinates& a, coordinates const& b) {
  return coordinates{a.x = a.x + b.x, a.y = a.y + b.y};
}
coordinates operator-(coordinates const& a, coordinates const& b) {
  return coordinates{a.x - b.x, a.y - b.y};
}
coordinates operator*(coordinates const& a, float const& b) {
  return coordinates{a.x * b, a.y * b};
}
coordinates operator*=(coordinates& a, float const& b) {
  return coordinates{a.x = a.x * b, a.y = a.y * b};
}
coordinates operator/(coordinates const& a, float const& b) {
  return coordinates{a.x / b, a.y / b};
}

void Flock::add_boids(values const& val) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> v_rand(-val.velocity_default,
                                          val.velocity_default);
  std::uniform_real_distribution<> p_rand(50, val.box_length - 50);

  boid new_boid{};
  for (int j = 0; j != val.n_boids; ++j) {
    new_boid.v.x = v_rand(gen);
    new_boid.v.y = v_rand(gen);
    new_boid.p.x = p_rand(gen);
    new_boid.p.y = p_rand(gen);

    flock_.push_back(new_boid);
  }
}
void Flock::update_flock(Flock& flock, values const& val, int const& fps,
                         int const& step) {
  if (step % val.velocity_step_update == 0) {
    update_velocity(flock, val);
  }
  update_position(flock, fps);
}

float Flock::velocity_mean(int const& n_boids) {
  float v_sum = std::accumulate(
      flock_.begin(), flock_.end(), 0.,
      [](float const a, boid const b) { return a + std::hypot(b.v.x, b.v.y); });
  return v_sum / n_boids;
}

float Flock::d_separation_mean() {
  float p_sum{};
  int i{};
  auto fbegin = flock_.begin();
  auto fend = flock_.end();
  for (auto it1 = fbegin; it1 != fend; ++it1) {
    for (auto it2 = std::next(it1); it2 != flock_.end(); ++it2, ++i) {
      p_sum += std::hypot(it1->p.x - it2->p.x, it1->p.y - it2->p.y);
    }
  }

  return p_sum / i;
}

coordinates Flock::center_mass(int const& n_boids) const {
  coordinates init{};
  coordinates sum = std::accumulate(
      flock_.begin(), flock_.end(), init,
      [](coordinates const& a, boid const& b) { return a + b.p; });
  return sum / (n_boids - 1);
}

coordinates Flock::velocity_cm(int const& n_boids) {
  coordinates init{};
  coordinates v_sum = std::accumulate(
      flock_.begin(), flock_.end(), init,
      [](coordinates const& a, boid const& b) { return a + b.v; });
  return v_sum / n_boids;
}

std::vector<boid>::iterator Flock::begin() { return flock_.begin(); }
std::vector<boid>::iterator Flock::end() { return flock_.end(); }
int Flock::size() const { return flock_.size(); }

float stddev(std::vector<float> const& hypotenuse) {
  float sum = std::accumulate(hypotenuse.begin(), hypotenuse.end(), 0.);
  float h_mean = sum / hypotenuse.size();
  sum = std::accumulate(hypotenuse.begin(), hypotenuse.end(), 0.,
                        [=](float const a, float const b) {
                          return (a + std::pow(b - h_mean, 2));
                        });
  return std::sqrt(sum / (hypotenuse.size() * (hypotenuse.size() - 1)));
}