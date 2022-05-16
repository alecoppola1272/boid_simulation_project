#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "simulation.hpp"

#include "doctest.h"

TEST_CASE("Testing size") {
    flock f{std::vector<boids> {}, double{2.}};
    std::vector<boids> boid_{};
    boid_[1] = {0., 0., 2., 3., 0.1};
    boid_[2] = {0., 0., 4., 5., 0.1};
    boid_[3] = {0., 0., 3., 2., 0.1};
    CHECK(f.size() == (3));

}