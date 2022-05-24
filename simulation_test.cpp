#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "simulation.hpp"

#include "doctest.h"

TEST_CASE("Mass Center") {
  boid b1{0., 0., 2., 3., .5};
  boid b2{0., 0., 3., 5., .5};
  flock f{{b1, b2}, 3.};
  flock::CM cm = f.mass_center();

  CHECK(cm.x_cm == (5.));
  CHECK(cm.y_cm == (8.));
}