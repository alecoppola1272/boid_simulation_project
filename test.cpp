#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "evolve.hpp"

TEST_CASE("operator+") {
  coordinates a{2., 3.};
  coordinates b{4., 5.};

  coordinates c = a + b;

  CHECK(c.x == 6);
  CHECK(c.y == 8);
}

TEST_CASE("Struct") {
  boid b{
      1.,
      2.,
      3.,
      4.,
  };

  CHECK(b.p.x == 1.);
  CHECK(b.p.y == 2.);
  CHECK(b.v.x == 3.);
  CHECK(b.v.y == 4.);
}

TEST_CASE("Separation rule") {
  SUBCASE("Two Boids") {
    values val;
    val.separation_factor = 0.3;

    boid b1{0., 0., 0., 0.};
    boid b2{0.2, 0.1, 0., 0.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2};

    coordinates v1 = separation_velocity(neighbors, it1, val);

    CHECK(v1.x == (-0.6));
    CHECK(v1.y == (-0.6));
  }
}