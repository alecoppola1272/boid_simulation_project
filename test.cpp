#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "simulation.hpp"

#include "doctest.h"

// TEST_CASE("Mass Center") {
//   SUBCASE("Two Boids") {
//     boid b1{0., 0., 2., 3.};
//     boid b2{0., 0., 3., 5.};
//     flock f{{b1, b2}, 3.};
//     flock::CM cm = f.mass_center();

//     CHECK(cm.x_cm == (5.));
//     CHECK(cm.y_cm == (8.));
//   }
  
//   SUBCASE("Three Boids") {
//     boid b1{0., 0., 2., 3.};
//     boid b2{0., 0., 3., 5.};
//     boid b3{0., 0., 7., 2.};
//     flock f{{b1, b2, b3}, 3.};
//     flock::CM cm = f.mass_center();
//     CHECK(cm.x_cm == (6.));
//     CHECK(cm.y_cm == (5.));
//   }
// }

// TEST_CASE("Separation Rule") {
//   SUBCASE("Two Boids") {
//     boid b1{2., 3., 2., 3.};
//     boid b2{1., 2., 3., 5.};
//     flock f{{b1, b2}, 3.};
//     flock::separation_v v1 = f.separation();
//     CHECK(v1.vx_1 == (-0.5));
//     CHECK(v1.vy_1 == (-1.));
//   }
// }