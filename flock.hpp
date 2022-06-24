#ifndef FLOCK_HPP
#define FLOCK_HPP

struct position {
  double x;
  double y;
};

struct velocity {
  double x;
  double y;
};

struct coordinates {
  position p;
  velocity v;
};

class Flock {
  std::vector<coordinates> flock_;

 public:
  Flock(std::vector<coordinates> flock) : flock_{flock} {}

  auto add_boids(values const& val) {
    coordinates new_boid;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> v_rand(1., val.velocity_default);
    std::uniform_real_distribution<> p_rand(1., val.box_length - 1);

    for (int j = 0; j != val.n_boids; ++j) {
      new_boid.v.x = v_rand(gen);
      new_boid.v.y = v_rand(gen);

      new_boid.p.x = p_rand(gen);
      new_boid.p.y = p_rand(gen);

      flock_.push_back(new_boid);
    }
    return flock_;
  }

  auto center_mass(int n_boids) {
    position cm;
    position sum;
    sum.x = 0;
    sum.y = 0;

    for (auto it = flock_.begin(); it != flock_.end(); ++it) {
      sum.x += it->p.x;
      sum.y += it->p.y;
    };

    // sum = std::accumulate(flock_.begin(), flock_.end(), sum,
    //                       [](coordinates const& first, coordinates const&
    //                       second) {
    //                         position result;
    //                         result.x = first.p.x + second.p.x;
    //                         result.y = first.p.y + second.p.y;
    //                         return result;
    //                       });

    cm.x = sum.x / (n_boids - 1);
    cm.y = sum.y / (n_boids - 1);

    return cm;
  }
  
  auto velocity_mean(int n_boids) {
    velocity vm;
    velocity sum;
    sum.x = 0;
    sum.y = 0;

    for (auto it = flock_.begin(); it != flock_.end(); ++it) {
      sum.x += it->v.x;
      sum.y += it->v.y;
    };

    vm.x = sum.x / n_boids;
    vm.y = sum.y / n_boids;

    return vm;
  }

  auto begin() { return flock_.begin(); }
  auto end() { return flock_.end(); }
};

#endif