#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "flock.cpp"

TEST_CASE("Coordinates operator") {
  coordinates a{2., 3.};
  coordinates b{4., 5.};
  float x{10};

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
  CHECK(f.x == doctest::Approx(0.2));
  CHECK(f.y == doctest::Approx(0.3));
}
TEST_CASE("+= Overload") {
  coordinates g{10., 9.};
  coordinates l{3., 9.};
  l += g;

  CHECK(l.x == 13.);
  CHECK(l.y == 18.);
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
    coordinates v1 = separation_velocity({it1, it2}, *it1, val);

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
    coordinates v1 = separation_velocity({it1, it2, it3}, *it1, val);

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
    coordinates v2 = alignment_velocity({it2}, *it1, val);

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
    coordinates v2 = alignment_velocity({it2, it3}, *it1, val);

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
    coordinates v3 = coesion_velocity({it1, it2}, *it1, val);

    CHECK(v3.x == doctest::Approx(0.05));
    CHECK(v3.y == doctest::Approx(0.05));
  }

  SUBCASE("Three Boids") {
    values val;
    val.coesion_factor = 0.5;
    val.n_boids = 3;

    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    boid b3{0.5, 0.6, 0., 0.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    coordinates v3 = coesion_velocity({it1, it2, it3}, *it1, val);

    CHECK(v3.x == doctest::Approx(0.1));
    CHECK(v3.y == doctest::Approx(0.1));
  }
}

TEST_CASE("Center Mass") {
  SUBCASE("Two boids") {
    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    Flock f{{b1, b2}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    coordinates cm = center_mass({it1, it2});

    CHECK(cm.x == doctest::Approx(0.2));
    CHECK(cm.y == doctest::Approx(0.3));
  }

  SUBCASE("Three Boids") {
    boid b1{0.1, 0.2, 0., 0.};
    boid b2{0.3, 0.4, 0., 0.};
    boid b3{0.5, 0.6, 0., 0.};
    Flock f{{b1, b2, b3}};

    auto it1 = f.begin();
    auto it2 = std::next(it1);
    auto it3 = std::next(it2);
    coordinates cm = center_mass({it1, it2, it3});

    CHECK(cm.x == doctest::Approx(0.3));
    CHECK(cm.y == doctest::Approx(0.4));
  }
}

TEST_CASE("Velocity edge") {
  SUBCASE("Boid left side outwards") {
    values val;
    Flock f{{{1, 2, -3, -4}}};
    auto it1 = f.begin();
    it1->v += velocity_edge(*it1, val);

    CHECK(it1->v.x == doctest::Approx(-1.515));
    CHECK(it1->v.y == doctest::Approx(-2.04));
  }
  SUBCASE("Boid left side inwards") {
    values val;
    Flock f{{{1, 2, 3, 4}}};
    auto it1 = f.begin();
    it1->v += velocity_edge(*it1, val);

    CHECK(it1->v.x == doctest::Approx(4.485));
    CHECK(it1->v.y == doctest::Approx(5.96));
  }
  SUBCASE("Boid right side outwards") {
    values val;
    Flock f{{{599, 598, 3, 4}}};
    auto it1 = f.begin();
    it1->v += velocity_edge(*it1, val);

    CHECK(it1->v.x == doctest::Approx(1.515));
    CHECK(it1->v.y == doctest::Approx(2.04));
  }
  SUBCASE("Boid right side inwards") {
    values val;
    Flock f{{{599, 598, -3, -4}}};
    auto it1 = f.begin();
    it1->v += velocity_edge(*it1, val);

    CHECK(it1->v.x == doctest::Approx(-4.485));
    CHECK(it1->v.y == doctest::Approx(-5.96));
  }
  SUBCASE("Boid inside") {
    values val;
    Flock f{{{300, 300, 3, -4}}};
    auto it1 = f.begin();
    velocity_edge(*it1, val);

    CHECK(it1->v.x == doctest::Approx(3));
    CHECK(it1->v.y == doctest::Approx(-4));
  }
}

TEST_CASE("Velocity limit") {
  values val;
  Flock f{{{0, 0, 100, -200}}};
  auto it1 = f.begin();
  it1->v = velocity_limit(it1->v, val);

  CHECK(std::hypot(it1->v.x, it1->v.y) == doctest::Approx(60));
}

TEST_CASE("Flock add boid") {
  values val;
  val.n_boids = 3;
  Flock f{};
  f.add_boids(val);

  auto it1 = f.begin();
  auto it2 = std::next(it1);
  auto it3 = std::next(it2);

  CHECK(f.size() == 3);
  CHECK(it1 != it2);
  CHECK(it2 != it3);
  CHECK(it3 != it1);
}

TEST_CASE("Flock velocity cm") {
  boid b1{0., 0., 1., 2.};
  boid b2{0., 0., -3., -4.};
  boid b3{0., 0., 5., 8.};
  Flock f{{b1, b2, b3}};
  coordinates v_cm = f.velocity_cm(3);

  CHECK(v_cm.x == doctest::Approx(1.));
  CHECK(v_cm.y == doctest::Approx(2.));
}

TEST_CASE("Flock mean velocity") {
  boid b1{0., 0., 1., 2.};
  boid b2{0., 0., -3., -4.};
  boid b3{0., 0., 5., 6.};
  Flock f{{b1, b2, b3}};
  float vm = f.velocity_mean(3);

  CHECK(vm == doctest::Approx(5.01544));
}

TEST_CASE("Flock distance separation mean") {
  boid b1{1., 2., 0., 0.};
  boid b2{3., 4., 0., 0.};
  boid b3{5., 6., 0., 0.};
  Flock f{{b1, b2, b3}};
  float dsm = f.d_separation_mean();

  CHECK(dsm == doctest::Approx(3.7712));
}

TEST_CASE("Boid vision") {
  SUBCASE("Boid in movimento") {
    values val;

    boid b1{0., 0., 1., 0.};
    boid b2{1., 0., 0., 0.};
    boid b3{0., 2., 0., 0.};
    boid b4{-3., 0., 0., 0.};
    boid b5{0., -4., 0., 0.};
    Flock f{{b1, b2, b3, b4, b5}};

    auto it1 = f.begin();
    std::vector<std::vector<boid>::iterator> neighbors{};
    int i{};

    auto fbegin = f.begin();
    auto fend = f.end();
    for (auto it2 = std::next(fbegin); it2 != fend; ++it2) {
      if (boid_vision(*it1, *it2, val.boid_blind_angle)) {
        ++i;
      }
    }

    CHECK(i == 3);
  }
  SUBCASE("Boid fermo") {
    values val;
    boid b1{0., 0., 0., 0.};
    boid b2{1., 0., 0., 0.};
    boid b3{0., 2., 0., 0.};
    boid b4{-3., 0., 0., 0.};
    boid b5{0., -4., 0., 0.};
    Flock f{{b1, b2, b3, b4, b5}};

    auto it1 = f.begin();
    std::vector<std::vector<boid>::iterator> neighbors{};
    int i{};

    auto fbegin = f.begin();
    auto fend = f.end();
    for (auto it2 = std::next(fbegin); it2 != fend; ++it2) {
      if (boid_vision(*it1, *it2, val.boid_blind_angle)) {
        ++i;
      };
    }

    CHECK(i == 4);
  }
}

TEST_CASE("Checking neighbors") {
  values val;
  boid b1{0., 0., 1., 0.};
  boid b2{3., 4., 1., 0.};
  boid b3{5., 6., 1., 0.};
  boid b4{70., 80., 1., 0.};
  Flock f{{b1, b2, b3, b4}};

  auto it1 = f.begin();
  std::vector<std::vector<boid>::iterator> neighbors{};
  neighbors = checking_neighbors(f, it1, val);

  CHECK(neighbors.size() == 2);
}

TEST_CASE("Update position") {
  values val;
  boid b1{1., 2., 1., 1.};
  boid b2{3., 4., 2., 2.};
  boid b3{5., 6., 3., 3.};
  boid b4{70., 80., 4., 4.};
  boid b5{110., 120., 5., 5.};
  Flock f{{b1, b2, b3, b4, b5}};

  update_position(f, 30);

  auto it1 = f.begin();
  auto it2 = std::next(it1);
  auto it3 = std::next(it2);
  auto it4 = std::next(it3);
  auto it5 = std::next(it4);

  CHECK(it1->p.x == doctest::Approx(1.0333333333));
  CHECK(it1->p.y == doctest::Approx(2.0333333333));
  CHECK(it2->p.x == doctest::Approx(3.0666666667));
  CHECK(it2->p.y == doctest::Approx(4.0666666667));
  CHECK(it3->p.x == doctest::Approx(5.1));
  CHECK(it3->p.y == doctest::Approx(6.1));
  CHECK(it4->p.x == doctest::Approx(70.1333333333));
  CHECK(it4->p.y == doctest::Approx(80.1333333333));
  CHECK(it5->p.x == doctest::Approx(110.1666666667));
  CHECK(it5->p.y == doctest::Approx(120.1666666667));
}

TEST_CASE("Standard deviation") {
  boid b1{1., 2., 1., 2.};
  boid b2{3., 4., 3., 4.};
  boid b3{5., 6., 5., 6.};
  boid b4{70., 80., 7., 8.};
  boid b5{110., 120., 9., 10.};

  float stdpx = stddev({b1.p.x, b2.p.x, b3.p.x, b4.p.x, b5.p.x});
  float stdpy = stddev({b1.p.y, b2.p.y, b3.p.y, b4.p.y, b5.p.y});
  float stdvx = stddev({b1.v.x, b2.v.x, b3.v.x, b4.v.x, b5.v.x});
  float stdvy = stddev({b1.v.y, b2.v.y, b3.v.y, b4.v.y, b5.v.y});

  CHECK(stdpx == doctest::Approx(22.23825));
  CHECK(stdpy == doctest::Approx(24.35898));
  CHECK(stdvx == doctest::Approx(1.41421));
  CHECK(stdvy == doctest::Approx(1.41421));
}