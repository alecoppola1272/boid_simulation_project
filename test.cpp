#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "evolve.hpp"

TEST_CASE("coordinates operator") {
  coordinates a{2., 3.};
  coordinates b{4., 5.};
  double x{10};

  coordinates c = a + b;
  coordinates d = a - b;
  coordinates e = a * x;
  coordinates f = a / x;

  CHECK(c.x == 6);
  CHECK(c.y == 8);
  CHECK(d.x == -2);
  CHECK(d.y == -2);
  CHECK(e.x == 20);
  CHECK(e.y == 30);
  CHECK(f.x == 0.2);
  CHECK(f.y == 0.3);
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
  SUBCASE("Two boids") {
    values val;
    val.separation_factor = 10;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2};

    coordinates v1 = separation_velocity(neighbors, it1, val);

    CHECK(v1.x == doctest::Approx(-4.));
    CHECK(v1.y == doctest::Approx(-4.));
  }

  SUBCASE("Three boids") {
    values val;
    val.separation_factor = 10;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    boid b3{0.5, 0.6, 0., 0.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2, it3};

    coordinates v1 = separation_velocity(neighbors, it1, val);

    CHECK(v1.x == doctest::Approx(-7.));
    CHECK(v1.y == doctest::Approx(-7.));
  }
}

TEST_CASE("Alignment rule") {
  SUBCASE("Two boids") {
    values val;
    val.alignment_factor = 0.5;
    val.n_boids = 2;

    boid b1{0., 0., 1., 2.};
    boid b2{0., 0., 3., 4.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2};

    coordinates v2 = alignment_velocity(neighbors, it1, val);

    CHECK(v2.x == 2.);
    CHECK(v2.y == 2.);
  }

  SUBCASE("Three boids") {
    values val;
    val.alignment_factor = 0.5;
    val.n_boids = 3;

    boid b1{0., 0., 1., 2.};
    boid b2{0., 0., 3., 4.};
    boid b3{0., 0., 5., 6.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2, it3};

    coordinates v2 = alignment_velocity(neighbors, it1, val);

    CHECK(v2.x == 1.25);
    CHECK(v2.y == 1.5);
  }
}

TEST_CASE("Coesion rule") {
  SUBCASE("Two boids") {
    values val;
    val.coesion_factor = 0.5;
    val.n_boids = 2;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2};

    coordinates v3 = coesion_velocity(neighbors, it1, val);

    CHECK(v3.x == 1);
    CHECK(v3.y == 1);
  }

  SUBCASE("Three Boids") {
    values val;
    val.coesion_factor = 0.5;
    val.n_boids = 2;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    boid b3{0.5, 0.6, 0., 0.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2, it3};

    coordinates v3 = coesion_velocity(neighbors, it1, val);

    CHECK(v3.x == 1);
    CHECK(v3.y == 1);
  }
}

TEST_CASE("Center Mass") {
  SUBCASE("Two boids") {
    values val;
    val.coesion_factor = 0.5;
    val.n_boids = 2;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2};

    coordinates v3 = coesion_velocity(neighbors, it1, val);

    CHECK(v3.x == 1);
    CHECK(v3.y == 1);
  }

  SUBCASE("Three Boids") {
    values val;
    val.coesion_factor = 0.5;
    val.n_boids = 2;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    boid b3{0.5, 0.6, 0., 0.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2, it3};

    coordinates v3 = coesion_velocity(neighbors, it1, val);

    CHECK(v3.x == 1);
    CHECK(v3.y == 1);
  }
}  // n_boids, neighbors

TEST_CASE("Velocity limit"){}

TEST_CASE("Velocity sum"){}

TEST_CASE("Flock add boid"){}

TEST_CASE("Flock velocity mean"){}

TEST_CASE("Flock distance separation mean"){}

TEST_CASE("Dial control"){}

TEST_CASE("Boid vision"){}

TEST_CASE("Checking neighbors"){}

TEST_CASE("Update velocity"){}

TEST_CASE("Position limit"){}

TEST_CASE("Update position"){}

TEST_CASE("Update Flock"){}