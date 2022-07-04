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
    val.separation_factor = 0.5;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2};

    coordinates v1 = separation_velocity(neighbors, it1, val);

    CHECK(v1.x == doctest::Approx(-0.1));
    CHECK(v1.y == doctest::Approx(-0.1));
  }

  SUBCASE("Three boids") {
    values val;
    val.separation_factor = 0.5;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    boid b3{0.5, 0.6, 0., 0.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2, it3};

    coordinates v1 = separation_velocity(neighbors, it1, val);

    CHECK(v1.x == doctest::Approx(-0.3));
    CHECK(v1.y == doctest::Approx(-0.3));
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

    CHECK(v2.x == doctest::Approx(1.));
    CHECK(v2.y == doctest::Approx(1.));
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

    CHECK(v2.x == doctest::Approx(1.75));
    CHECK(v2.y == doctest::Approx(2.));
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

    CHECK(v3.x == doctest::Approx(0.15));
    CHECK(v3.y == doctest::Approx(0.2));
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

    CHECK(v3.x == doctest::Approx(0.4));
    CHECK(v3.y == doctest::Approx(0.5));
  }
}

TEST_CASE("Center Mass") {
  SUBCASE("Two boids") {
    values val;
    val.n_boids = 2;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2};

    coordinates cm = center_mass(val.n_boids, neighbors);

    CHECK(cm.x == doctest::Approx(0.4));
    CHECK(cm.y == doctest::Approx(0.6));
  }

  SUBCASE("Three Boids") {
    values val;
    val.n_boids = 2;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    boid b3{0.5, 0.6, 0., 0.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    std::vector<std::vector<boid>::iterator> const& neighbors{it1, it2, it3};

    coordinates cm = center_mass(val.n_boids, neighbors);

    CHECK(cm.x == doctest::Approx(0.9));
    CHECK(cm.y == doctest::Approx(1.2));
  }
}

TEST_CASE("Velocity edge") {
  SUBCASE("Boid lato sinistro verso fuori") {  // tradurre
    values val;
    boid b1{1, 2, -3, -4};
    Flock f{{b1}};
    auto it1 = f.begin();
    coordinates v4 = velocity_edge(it1, val);

    CHECK(v4.x == doctest::Approx(54.));
    CHECK(v4.y == doctest::Approx(64.));
  }

  SUBCASE("Boid lato sinistro verso dentro") {  // tradurre
    values val;
    boid b1{1, 2, 3, 4};
    Flock f{{b1}};
    auto it1 = f.begin();
    coordinates v4 = velocity_edge(it1, val);

    CHECK(v4.x == doctest::Approx(-54.));  // !
    CHECK(v4.y == doctest::Approx(-64.));
  }
  SUBCASE("Boid lato destro verso fuori") {}   // tradurre
  SUBCASE("Boid lato destro verso dentro") {}  // tradurre
}

TEST_CASE("Velocity limit") {
  values val;
  boid b1{0, 0, 100, -200};
  Flock f{{b1}};
  auto it1 = f.begin();
  velocity_limit(it1, val);

  CHECK(it1->v.x == doctest::Approx(20));
  CHECK(it1->v.y == doctest::Approx(-20));
}

TEST_CASE("Flock add boid") {
  values val;
  val.n_boids = 3;

  Flock f{{}};
  f.add_boids(val);

  auto it1 = f.begin();
  auto it2 = std::next(it1);
  auto it3 = std::next(it2);

  CHECK(f.size() == 3);
  CHECK(it1 != it2);
  CHECK(it2 != it3);
  CHECK(it3 != it1);
}

TEST_CASE("Flock velocity mean") {
  values val;
  val.n_boids = 3;

  boid b1{0., 0., 1., 2.};
  boid b2{0., 0., -3., -4.};
  boid b3{0., 0., 5., 8.};
  Flock f{{b1, b2, b3}};

  coordinates vm = f.velocity_mean(val.n_boids);

  CHECK(vm.x == doctest::Approx(1.));
  CHECK(vm.y == doctest::Approx(2.));
}

TEST_CASE("Flock distance separation mean") {
  values val;
  val.n_boids = 3;

  boid b1{1., 2., 0., 0.};
  boid b2{3., 4., 0., 0.};
  boid b3{5., 6., 0., 0.};
  Flock f{{b1, b2, b3}};

  coordinates dsm = f.d_separation_mean();

  CHECK(dsm.x == doctest::Approx(2.6666666667));
  CHECK(dsm.y == doctest::Approx(2.6666666667));
}

TEST_CASE("Boid vision") {
  values val;

  boid b1{1., 2., 1., 0.};
  boid b2{3., 4., 0., 1.};
  boid b3{5., 6., -1., 0.};
  boid b4{7., 8., 0., -1.};
  Flock f{{b1, b2, b3, b4}};

  auto it1 = f.begin();
  std::vector<std::vector<boid>::iterator> neighbors{};

  for (auto it2 = std::next(f.begin()); it2 != f.end(); ++it2) {
    boid_vision(it1, it2, val.boid_vision_angle, neighbors);
  }

  CHECK(neighbors.size() == 2);
}

TEST_CASE("Checking neighbors") {}

TEST_CASE("Update velocity") {}

TEST_CASE("Position limit") {}

TEST_CASE("Update position") {}

TEST_CASE("Update Flock") {}