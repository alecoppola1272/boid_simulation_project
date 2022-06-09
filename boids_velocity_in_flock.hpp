#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <array>
#include <cmath>
#include <numeric>
#include <vector>

constexpr int spatial_dimension{2};
constexpr double velocity_default{10.};
constexpr double velocity_max{20.};
constexpr double distance_separation{5.};
double separation_factor;
double alignment_factor;
double coesion_factor;

struct coordinates {
  std::array<double, spatial_dimension> position;
  std::array<double, spatial_dimension> velocity;
};

class Flock {
    std::vector<coordinates> flock;


}



#endif