#include <gtest/gtest.h>
#include <cmath>
#include "usv_control/planning/obstacle_tracker.hpp"

// ---------- bodyToNed ----------
TEST(BodyToNed, FacingNorthIsIdentityPlusOffset) {
  Point2D p = bodyToNed(10.0, 20.0, 0.0, 5.0, 2.0);
  EXPECT_NEAR(p.x, 15.0, 1e-9);   // proa = Norte
  EXPECT_NEAR(p.y, 22.0, 1e-9);   // estribor = Este
}

TEST(BodyToNed, FacingEast) {
  Point2D p = bodyToNed(0.0, 0.0, M_PI / 2, 5.0, 2.0);
  EXPECT_NEAR(p.x, -2.0, 1e-9);   // estribor apunta al Sur
  EXPECT_NEAR(p.y, 5.0, 1e-9);    // proa apunta al Este
}

TEST(BodyToNed, FacingSouth) {
  Point2D p = bodyToNed(0.0, 0.0, M_PI, 5.0, 0.0);
  EXPECT_NEAR(p.x, -5.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
}

// ---------- ObstacleTracker ----------
TEST(ObstacleTracker, NewDetectionsAreAdded) {
  ObstacleTracker t(1.0, 0.0);
  t.update({{0.0, 0.0, 0.15, ""}, {10.0, 0.0, 0.15, ""}}, 0.0);
  EXPECT_EQ(t.size(), 2u);
}

TEST(ObstacleTracker, SameUuidUpdatesInsteadOfDuplicating) {
  ObstacleTracker t(1.0, 0.0);
  t.update({{0.0, 0.0, 0.15, "a"}}, 0.0);
  t.update({{5.0, 0.0, 0.15, "a"}}, 1.0);        // lejos, pero mismo uuid
  ASSERT_EQ(t.size(), 1u);
  EXPECT_DOUBLE_EQ(t.obstacles()[0].x, 5.0);
}

TEST(ObstacleTracker, NearbyWithoutUuidMerges) {
  ObstacleTracker t(1.0, 0.0);
  t.update({{0.0, 0.0, 0.15, ""}}, 0.0);
  t.update({{0.3, 0.2, 0.15, ""}}, 1.0);         // ruido de percepción
  EXPECT_EQ(t.size(), 1u);
}

TEST(ObstacleTracker, FarWithoutUuidIsNew) {
  ObstacleTracker t(1.0, 0.0);
  t.update({{0.0, 0.0, 0.15, ""}}, 0.0);
  t.update({{3.0, 0.0, 0.15, ""}}, 1.0);
  EXPECT_EQ(t.size(), 2u);
}

TEST(ObstacleTracker, DifferentUuidsDoNotMergeEvenIfClose) {
  ObstacleTracker t(1.0, 0.0);
  t.update({{0.0, 0.0, 0.15, "a"}, {0.5, 0.0, 0.15, "b"}}, 0.0);  // boyas de una compuerta
  EXPECT_EQ(t.size(), 2u);
}

TEST(ObstacleTracker, NoTimeoutKeepsForever) {
  ObstacleTracker t(1.0, 0.0);
  t.update({{0.0, 0.0, 0.15, ""}}, 0.0);
  EXPECT_FALSE(t.prune(1e6));
  EXPECT_EQ(t.size(), 1u);
}

TEST(ObstacleTracker, TimeoutForgetsOldOnly) {
  ObstacleTracker t(1.0, 5.0);
  t.update({{0.0, 0.0, 0.15, ""}}, 0.0);
  t.update({{10.0, 0.0, 0.15, ""}}, 4.0);
  EXPECT_TRUE(t.prune(6.0));                       // la 1a lleva 6 s sin verse
  ASSERT_EQ(t.size(), 1u);
  EXPECT_DOUBLE_EQ(t.obstacles()[0].x, 10.0);
}

TEST(ObstacleTracker, SeeingAgainResetsTimeout) {
  ObstacleTracker t(1.0, 5.0);
  t.update({{0.0, 0.0, 0.15, ""}}, 0.0);
  t.update({{0.1, 0.0, 0.15, ""}}, 4.0);
  EXPECT_FALSE(t.prune(6.0));
  EXPECT_EQ(t.size(), 1u);
}
