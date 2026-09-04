#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "belugaslam_core/submap.hpp"

namespace {

Sophus::SE2d IdentityPose() {
  return Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d::Zero()};
}

TEST(SubmapTest, FrozenDistanceFieldAndIdSurviveClone) {
  auto submap = std::make_shared<Submap>(7, IdentityPose(), 20, 20, 0.1);
  submap->mutable_grid().at(10, 10) = 5.0F;
  submap->finish();

  EXPECT_EQ(submap->id(), 7U);
  EXPECT_NEAR(submap->distance_at(0.05, 0.05), 0.0, 1.0e-6);
  EXPECT_NEAR(submap->distance_at(0.25, 0.05), 0.2, 0.03);

  const auto clone = submap->clone();
  EXPECT_EQ(clone->id(), submap->id());
  EXPECT_NEAR(clone->distance_at(0.25, 0.05), 0.2, 0.03);
}

TEST(LogOddsGridTest, GrowPreservesWorldCoordinatesOfExistingCells) {
  LogOddsGrid grid(20, 20, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-1.0, -1.0}});
  // Cell (5, 6) spans local x in [-0.5, -0.4), y in [-0.4, -0.3).
  grid.at(5, 6) = 3.0F;

  // Ask for a box that overflows the low corner and the high corner at once.
  EXPECT_TRUE(grid.grow_to_include(-2.5, -2.5, 3.0, 3.0));
  EXPECT_GT(grid.width(), 20);
  EXPECT_GT(grid.height(), 20);

  const auto index = [&grid](double x, double y) {
    const int gx = static_cast<int>(std::floor((x - grid.origin_x()) / grid.resolution()));
    const int gy = static_cast<int>(std::floor((y - grid.origin_y()) / grid.resolution()));
    return std::pair<int, int>{gx, gy};
  };
  const auto [gx, gy] = index(-0.45, -0.35);
  EXPECT_FLOAT_EQ(grid.at(gx, gy), 3.0F);

  // The requested box is now addressable and the rest of the grid is still unknown.
  const auto [lx, ly] = index(-2.5, -2.5);
  EXPECT_GE(lx, 0);
  EXPECT_GE(ly, 0);
  EXPECT_FLOAT_EQ(grid.at(lx, ly), 0.0F);
  EXPECT_EQ(grid.data().size(), static_cast<std::size_t>(grid.width()) * grid.height());
}

// The invariant that makes growth safe: an existing cell keeps its metric coordinates,
// so p_local(cell) is the same before and after. Checked over every cell rather than a
// sample, because an off-by-one in either the origin shift or the row copy would still
// leave most cells looking right.
TEST(LogOddsGridTest, GrowPreservesEveryCellMetricCoordinateAndValue) {
  constexpr int kWidth = 17;   // deliberately not a multiple of the growth chunk
  constexpr int kHeight = 23;
  constexpr double kResolution = 0.05;
  LogOddsGrid grid(kWidth, kHeight, kResolution,
                   Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-0.37, 0.11}});

  // A distinct non-zero value per cell, plus the metric centre it must keep.
  struct Sample { double x, y; float value; };
  std::vector<Sample> samples;
  samples.reserve(static_cast<std::size_t>(kWidth) * kHeight);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const float value = static_cast<float>(1 + y * kWidth + x);
      grid.at(x, y) = value;
      samples.push_back({grid.origin_x() + (x + 0.5) * kResolution,
                         grid.origin_y() + (y + 0.5) * kResolution, value});
    }
  }

  // Grow asymmetrically: far past the low corner, barely past the high corner.
  ASSERT_TRUE(grid.grow_to_include(-3.1, -2.4, 0.9, 1.5));

  const auto lookup = [&grid](double x, double y) {
    const int gx = static_cast<int>(std::floor((x - grid.origin_x()) / grid.resolution()));
    const int gy = static_cast<int>(std::floor((y - grid.origin_y()) / grid.resolution()));
    return std::pair<int, int>{gx, gy};
  };

  for (const auto& sample : samples) {
    const auto [gx, gy] = lookup(sample.x, sample.y);
    ASSERT_GE(gx, 0);
    ASSERT_GE(gy, 0);
    ASSERT_LT(gx, grid.width());
    ASSERT_LT(gy, grid.height());
    EXPECT_FLOAT_EQ(grid.at(gx, gy), sample.value)
        << "cell at (" << sample.x << ", " << sample.y << ") moved or changed value";
  }

  // Everything else must be unknown: no value may be duplicated by the row copy.
  const auto non_zero = std::count_if(
      grid.data().begin(), grid.data().end(), [](float v) { return v != 0.0F; });
  EXPECT_EQ(non_zero, static_cast<long>(samples.size()));
}

// Growing twice must compose: the second call cannot undo the first call's shift.
TEST(LogOddsGridTest, RepeatedGrowthKeepsTheInvariant) {
  LogOddsGrid grid(10, 10, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}});
  grid.at(2, 3) = 7.0F;
  const double marker_x = grid.origin_x() + 2.5 * 0.1;
  const double marker_y = grid.origin_y() + 3.5 * 0.1;

  ASSERT_TRUE(grid.grow_to_include(-1.5, -0.5, 0.5, 0.5));   // grows low side
  ASSERT_TRUE(grid.grow_to_include(-0.5, -0.5, 8.0, 9.0));   // grows high side

  const int gx = static_cast<int>(std::floor((marker_x - grid.origin_x()) / grid.resolution()));
  const int gy = static_cast<int>(std::floor((marker_y - grid.origin_y()) / grid.resolution()));
  EXPECT_FLOAT_EQ(grid.at(gx, gy), 7.0F);
}

// Cropping is the mirror of growth and must keep the same invariant: an observed cell
// keeps its metric coordinates, only the origin moves.
TEST(LogOddsGridTest, CropShrinksToTheObservedBoxAndKeepsCoordinates) {
  constexpr int kMargin = 5;
  LogOddsGrid grid(60, 60, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-3.0, -3.0}});
  // Two observed cells far from the borders; everything else is unknown.
  grid.at(20, 25) = 4.0F;
  grid.at(28, 31) = -1.0F;
  const double ax = grid.origin_x() + 20.5 * 0.1, ay = grid.origin_y() + 25.5 * 0.1;
  const double bx = grid.origin_x() + 28.5 * 0.1, by = grid.origin_y() + 31.5 * 0.1;

  ASSERT_TRUE(grid.crop_to_known_cells(kMargin));

  // Observed box is 9 x 7 cells, plus the margin on each side.
  EXPECT_EQ(grid.width(), 9 + 2 * kMargin);
  EXPECT_EQ(grid.height(), 7 + 2 * kMargin);
  EXPECT_EQ(grid.data().size(), static_cast<std::size_t>(grid.width()) * grid.height());

  const auto lookup = [&grid](double x, double y) {
    return std::pair<int, int>{
        static_cast<int>(std::floor((x - grid.origin_x()) / grid.resolution())),
        static_cast<int>(std::floor((y - grid.origin_y()) / grid.resolution()))};
  };
  const auto [gax, gay] = lookup(ax, ay);
  const auto [gbx, gby] = lookup(bx, by);
  EXPECT_FLOAT_EQ(grid.at(gax, gay), 4.0F);
  EXPECT_FLOAT_EQ(grid.at(gbx, gby), -1.0F);

  // Nothing else survived, and the margin really is unknown space.
  const auto non_zero = std::count_if(
      grid.data().begin(), grid.data().end(), [](float v) { return v != 0.0F; });
  EXPECT_EQ(non_zero, 2);
}

TEST(LogOddsGridTest, CropLeavesAnUnobservedGridAlone) {
  LogOddsGrid grid(20, 20, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-1.0, -1.0}});
  EXPECT_FALSE(grid.crop_to_known_cells(5));
  EXPECT_EQ(grid.width(), 20);
  EXPECT_EQ(grid.height(), 20);
}

TEST(LogOddsGridTest, CropIsANoOpWhenTheObservedBoxAlreadyFillsTheGrid) {
  LogOddsGrid grid(10, 10, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}});
  grid.at(0, 0) = 1.0F;
  grid.at(9, 9) = 1.0F;
  EXPECT_FALSE(grid.crop_to_known_cells(5));
  EXPECT_EQ(grid.width(), 10);
}

// The radial signature must be measured from the submap origin, local (0, 0), not from
// the centre of the cell array. Growth and cropping both move the array centre away from
// the origin, so an implementation using width * resolution / 2 would put this wall in a
// different bin -- and since the signature ranks loop closure candidates, that failure is
// silent. One wall cell at a known radius pins the convention down.
TEST(SubmapTest, RadialSignatureIsMeasuredFromTheOriginNotTheArrayCentre) {
  auto submap = std::make_shared<Submap>(5, IdentityPose(), 40, 40, 0.1);
  auto& grid = submap->mutable_grid();
  // Cell (32, 20) is centred at local (1.25, 0.05): radius 1.2510, i.e. bin 2 of 0.5 m.
  ASSERT_NEAR(grid.origin_x() + 32.5 * 0.1, 1.25, 1.0e-9);
  ASSERT_NEAR(grid.origin_y() + 20.5 * 0.1, 0.05, 1.0e-9);
  grid.at(32, 20) = 5.0F;

  submap->finish();

  const auto& signature = submap->radial_signature();
  ASSERT_EQ(signature.size(), 50U);
  EXPECT_DOUBLE_EQ(signature[2], 1.0);
  // Cropping leaves the single wall near the middle of the array, so an array-centred
  // implementation would land in bin 0 instead.
  EXPECT_DOUBLE_EQ(signature[0], 0.0);
  for (std::size_t bin = 0; bin < signature.size(); ++bin) {
    if (bin != 2) EXPECT_DOUBLE_EQ(signature[bin], 0.0) << "bin " << bin;
  }
}

// grow while active -> crop -> freeze -> derived structures, all in finish().
TEST(SubmapTest, FinishCropsBeforeBuildingTheDistanceField) {
  auto submap = std::make_shared<Submap>(3, IdentityPose(), 200, 200, 0.1);
  // One wall cell, reached only after the grid has grown well past it.
  submap->mutable_grid().at(100, 100) = 5.0F;
  const auto& grid = submap->grid();
  const double wall_x = grid.origin_x() + 100.5 * 0.1;
  const double wall_y = grid.origin_y() + 100.5 * 0.1;

  submap->finish();

  EXPECT_TRUE(submap->is_finished());
  EXPECT_LT(submap->grid().width(), 200);
  EXPECT_LT(submap->grid().height(), 200);
  // The distance field is sized to the cropped grid and still answers correctly.
  EXPECT_NEAR(submap->distance_at(wall_x, wall_y), 0.0, 1.0e-6);
  EXPECT_NEAR(submap->distance_at(wall_x + 0.2, wall_y), 0.2, 0.03);
}

TEST(LogOddsGridTest, GrowIsANoOpWhenTheBoxAlreadyFits) {
  LogOddsGrid grid(20, 20, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-1.0, -1.0}});
  const double origin_x = grid.origin_x();
  EXPECT_FALSE(grid.grow_to_include(-0.5, -0.5, 0.5, 0.5));
  EXPECT_EQ(grid.width(), 20);
  EXPECT_EQ(grid.height(), 20);
  EXPECT_DOUBLE_EQ(grid.origin_x(), origin_x);
}

TEST(SubmapTest, GrowingAnActiveSubmapDoesNotMoveItsGlobalPose) {
  const Sophus::SE2d pose{Sophus::SO2d{0.7}, Eigen::Vector2d{3.0, -2.0}};
  auto submap = std::make_shared<Submap>(1, pose, 20, 20, 0.1);
  submap->mutable_grid().grow_to_include(-5.0, -5.0, 5.0, 5.0);

  EXPECT_TRUE(submap->global_pose().translation().isApprox(pose.translation()));
  EXPECT_NEAR(submap->global_pose().so2().log(), pose.so2().log(), 1.0e-12);
}

// --- weighted mean pose, used to seed a hypothesis local pose ----------------------

TEST(WeightedMeanPoseTest, AveragesTranslationByWeight) {
  const std::vector<Sophus::SE2d> poses{
      Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{4.0, 8.0}}};
  const auto mean = weighted_mean_pose(poses, {3.0, 1.0});

  EXPECT_NEAR(mean.translation().x(), 1.0, 1.0e-12);
  EXPECT_NEAR(mean.translation().y(), 2.0, 1.0e-12);
}

// The reason theta needs a circular mean: the arithmetic mean of 179 and -179 degrees is
// 0, pointing the hypothesis in exactly the opposite direction to every particle in it.
TEST(WeightedMeanPoseTest, AveragesAngleCircularlyAcrossTheWrap) {
  const double kPi = Sophus::Constants<double>::pi();
  const std::vector<Sophus::SE2d> poses{
      Sophus::SE2d{Sophus::SO2d{kPi - 0.02}, Eigen::Vector2d::Zero()},
      Sophus::SE2d{Sophus::SO2d{-kPi + 0.02}, Eigen::Vector2d::Zero()}};
  const auto mean = weighted_mean_pose(poses, {1.0, 1.0});

  EXPECT_NEAR(std::abs(mean.so2().log()), kPi, 1.0e-9);
}

TEST(WeightedMeanPoseTest, FallsBackToAnUnweightedMeanWhenWeightsVanish) {
  const std::vector<Sophus::SE2d> poses{
      Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{2.0, 6.0}}};
  const auto mean = weighted_mean_pose(poses, {0.0, 0.0});

  EXPECT_NEAR(mean.translation().x(), 1.0, 1.0e-12);
  EXPECT_NEAR(mean.translation().y(), 3.0, 1.0e-12);
}

TEST(WeightedMeanPoseTest, IsIdentityWithNoPoses) {
  const auto mean = weighted_mean_pose({}, {});
  EXPECT_TRUE(mean.translation().isApprox(Eigen::Vector2d::Zero()));
  EXPECT_NEAR(mean.so2().log(), 0.0, 1.0e-12);
}

TEST(HypothesisTest, StartsWithoutALocalPoseSoItIsSeededOnce) {
  Hypothesis hypothesis;
  EXPECT_FALSE(hypothesis.has_local_pose);

  hypothesis.local_pose = Sophus::SE2d{Sophus::SO2d{0.5}, Eigen::Vector2d{1.0, 2.0}};
  hypothesis.has_local_pose = true;

  // A fork copies the parent wholesale; a spatial split then clears the flag so the
  // child re-seeds from its own cluster instead of inheriting this trajectory.
  Hypothesis child = hypothesis;
  EXPECT_TRUE(child.has_local_pose);
  EXPECT_NEAR(child.local_pose.translation().x(), 1.0, 1.0e-12);
  child.has_local_pose = false;
  EXPECT_TRUE(hypothesis.has_local_pose);
}

// --- dynamically sized derived views -----------------------------------------------

TEST(LogOddsGridTest, ResetChangesExtentAndClears) {
  LogOddsGrid grid(10, 10, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}});
  grid.at(5, 5) = 3.0F;

  grid.reset(7, 12, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-4.0, 2.0}});

  EXPECT_EQ(grid.width(), 7);
  EXPECT_EQ(grid.height(), 12);
  EXPECT_EQ(grid.data().size(), 7U * 12U);
  EXPECT_DOUBLE_EQ(grid.origin_x(), -4.0);
  EXPECT_DOUBLE_EQ(grid.origin_y(), 2.0);
  EXPECT_TRUE(std::all_of(grid.data().begin(), grid.data().end(),
                          [](float v) { return v == 0.0F; }));

  // Same extent: still cleared, no stale values left behind.
  grid.at(3, 3) = 9.0F;
  grid.reset(7, 12, grid.origin());
  EXPECT_FLOAT_EQ(grid.at(3, 3), 0.0F);
}

TEST(DynamicOccupancyGridTest, ResetAdoptsExtentAndFill) {
  DynamicOccupancyGrid grid;
  grid.reset(5, 4, 0.2, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{1.0, -1.0}}, -1);

  EXPECT_EQ(grid.width(), 5);
  EXPECT_EQ(grid.height(), 4);
  EXPECT_EQ(grid.data().size(), 20U);
  EXPECT_DOUBLE_EQ(grid.resolution(), 0.2);
  EXPECT_DOUBLE_EQ(grid.origin().translation().x(), 1.0);
  EXPECT_TRUE(std::all_of(grid.data().begin(), grid.data().end(),
                          [](std::int8_t v) { return v == -1; }));
}

TEST(SubmapGraphTest, BoundingBoxIsEmptyWithoutSubmaps) {
  SubmapList graph;
  double min_x = 99.0, min_y = 99.0, max_x = 99.0, max_y = 99.0;
  EXPECT_FALSE(graph.bounding_box(min_x, min_y, max_x, max_y));
  EXPECT_DOUBLE_EQ(min_x, 99.0);  // outputs untouched
}

TEST(SubmapGraphTest, BoundingBoxCoversAnAxisAlignedSubmap) {
  SubmapList graph;
  // 20 x 20 cells at 0.1 m, centred on the origin: local box [-1, 1] x [-1, 1].
  graph.active_submaps.push_back(std::make_shared<Submap>(0, IdentityPose(), 20, 20, 0.1));

  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  ASSERT_TRUE(graph.bounding_box(min_x, min_y, max_x, max_y));
  EXPECT_NEAR(min_x, -1.0, 1.0e-9);
  EXPECT_NEAR(max_x, 1.0, 1.0e-9);
  EXPECT_NEAR(min_y, -1.0, 1.0e-9);
  EXPECT_NEAR(max_y, 1.0, 1.0e-9);
}

// A rotated submap is the case that separates "transform four corners" from "transform
// two". At 45 degrees the extreme corners are the ones a two-corner version never sees,
// and the box would collapse to a point in x.
TEST(SubmapGraphTest, BoundingBoxUsesAllFourCornersOfARotatedSubmap) {
  SubmapList graph;
  const double kQuarterPi = std::atan(1.0);  // pi / 4
  graph.active_submaps.push_back(std::make_shared<Submap>(
      0, Sophus::SE2d{Sophus::SO2d{kQuarterPi}, Eigen::Vector2d{5.0, 0.0}}, 20, 20, 0.1));

  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  ASSERT_TRUE(graph.bounding_box(min_x, min_y, max_x, max_y));

  const double kHalfDiagonal = std::sqrt(2.0);  // corner (1, 1) rotated by 45 degrees
  EXPECT_NEAR(min_x, 5.0 - kHalfDiagonal, 1.0e-9);
  EXPECT_NEAR(max_x, 5.0 + kHalfDiagonal, 1.0e-9);
  EXPECT_NEAR(min_y, -kHalfDiagonal, 1.0e-9);
  EXPECT_NEAR(max_y, kHalfDiagonal, 1.0e-9);
}

TEST(SubmapGraphTest, BoundingBoxSpansHistoryAndActiveSubmaps) {
  SubmapList graph;
  graph.history.push_back(std::make_shared<Submap>(
      0, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-10.0, 0.0}}, 20, 20, 0.1));
  graph.active_submaps.push_back(std::make_shared<Submap>(
      1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{10.0, 3.0}}, 20, 20, 0.1));

  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  ASSERT_TRUE(graph.bounding_box(min_x, min_y, max_x, max_y));
  EXPECT_NEAR(min_x, -11.0, 1.0e-9);
  EXPECT_NEAR(max_x, 11.0, 1.0e-9);
  EXPECT_NEAR(min_y, -1.0, 1.0e-9);
  EXPECT_NEAR(max_y, 4.0, 1.0e-9);
}

// --- copy-on-write: shared frozen grids, independent poses -------------------------
//
// This is what makes the multi-hypothesis design affordable. Several hypotheses hold the
// same frozen submap, so its grid and distance field exist once no matter how many
// hypotheses there are, while each hypothesis still optimises its own copy of the pose.
// Growth, cropping and the active-submap cap all mutate grids, so these pin the property
// down rather than trusting that it survived.

TEST(SubmapCopyOnWriteTest, FinishedGridIsSharedWithClonesRatherThanCopied) {
  auto submap = std::make_shared<Submap>(1, IdentityPose(), 30, 30, 0.1);
  submap->mutable_grid().at(15, 15) = 5.0F;
  submap->finish();

  const auto clone = submap->clone();

  // The same grid object, not an equal one: one allocation for every hypothesis.
  EXPECT_EQ(&submap->grid(), &clone->grid());
  EXPECT_EQ(submap->grid().data().data(), clone->grid().data().data());
}

TEST(SubmapCopyOnWriteTest, ActiveGridIsCopiedSoHypothesesCanDiverge) {
  auto submap = std::make_shared<Submap>(1, IdentityPose(), 30, 30, 0.1);
  submap->mutable_grid().at(15, 15) = 1.0F;
  ASSERT_FALSE(submap->is_finished());

  const auto clone = submap->clone();
  EXPECT_NE(&submap->grid(), &clone->grid());

  // Writing through one must not be visible through the other.
  clone->mutable_grid().at(15, 15) = 4.0F;
  EXPECT_FLOAT_EQ(submap->grid().at(15, 15), 1.0F);
  EXPECT_FLOAT_EQ(clone->grid().at(15, 15), 4.0F);

  // Growth is a mutation too, and must not reach across the copy.
  clone->mutable_grid().grow_to_include(-10.0, -10.0, 10.0, 10.0);
  EXPECT_EQ(submap->grid().width(), 30);
  EXPECT_GT(clone->grid().width(), 30);
}

TEST(SubmapCopyOnWriteTest, AFinishedSubmapRefusesToBeMutated) {
  auto submap = std::make_shared<Submap>(1, IdentityPose(), 30, 30, 0.1);
  submap->mutable_grid().at(15, 15) = 5.0F;
  submap->finish();

  EXPECT_THROW((void)submap->mutable_grid(), std::runtime_error);
}

// Grids are shared, poses are not: this is what lets each hypothesis run its own pose
// graph optimisation over the same frozen maps.
TEST(SubmapCopyOnWriteTest, ClonesShareTheGridButOwnTheirPose) {
  auto submap = std::make_shared<Submap>(1, IdentityPose(), 30, 30, 0.1);
  submap->mutable_grid().at(15, 15) = 5.0F;
  submap->finish();

  const auto clone = submap->clone();
  clone->set_global_pose(Sophus::SE2d{Sophus::SO2d{0.3}, Eigen::Vector2d{4.0, -1.0}});

  EXPECT_EQ(&submap->grid(), &clone->grid());
  EXPECT_TRUE(submap->global_pose().translation().isApprox(Eigen::Vector2d::Zero()));
  EXPECT_NEAR(clone->global_pose().translation().x(), 4.0, 1.0e-12);
}

// make_room_for_new_submap() freezes submaps too; what it freezes must be shareable.
TEST(SubmapCopyOnWriteTest, SubmapsFrozenToMakeRoomAreSharedByClones) {
  SubmapList graph;
  auto first = std::make_shared<Submap>(0, IdentityPose(), 30, 30, 0.1);
  first->mutable_grid().at(15, 15) = 5.0F;
  graph.active_submaps.push_back(first);
  graph.active_submaps.push_back(std::make_shared<Submap>(1, IdentityPose(), 30, 30, 0.1));

  ASSERT_EQ(graph.make_room_for_new_submap(2).size(), 1U);
  ASSERT_TRUE(first->is_finished());

  const auto clone = first->clone();
  EXPECT_EQ(&first->grid(), &clone->grid());
}

// --- Cartographer insertion semantics ---------------------------------------------

namespace {
struct InsertionFixture {
  LogOddsGrid grid{40, 40, 0.1, Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{-2.0, -2.0}}};
  ScanInsertionParams params{};
  std::vector<int> hits, misses;

  void insert(const std::vector<std::pair<double, double>>& scan) {
    insert_scan_into_submap_grid(grid, IdentityPose(), scan, params, hits, misses);
  }
  float at(double x, double y) {
    const int gx = static_cast<int>(std::floor((x - grid.origin_x()) / grid.resolution()));
    const int gy = static_cast<int>(std::floor((y - grid.origin_y()) / grid.resolution()));
    return grid.at(gx, gy);
  }
};
}  // namespace

// A cell that is a return for one beam and merely crossed by another must stay a hit.
// Applying beam by beam would leave l_occ + l_free here instead of l_occ.
TEST(ScanInsertionTest, HitBeatsAMissFromAnotherBeamInTheSameCell) {
  InsertionFixture f;
  // Both beams run along +x. The first ends at 0.55 m; the second passes through that
  // same cell on its way to 1.05 m.
  f.insert({{0.55, 0.0}, {1.05, 0.0}});

  EXPECT_FLOAT_EQ(f.at(0.55, 0.0), f.params.l_occ);
  // The second beam's own return is a plain hit.
  EXPECT_FLOAT_EQ(f.at(1.05, 0.0), f.params.l_occ);
}

// A cell crossed by many beams must be counted free exactly once per scan.
TEST(ScanInsertionTest, SharedFreeCellIsUpdatedOncePerScan) {
  InsertionFixture f;
  // Four beams fanning out; all of them cross the cell just in front of the sensor.
  f.insert({{1.5, 0.0}, {1.5, 0.05}, {1.5, -0.05}, {1.5, 0.02}});

  EXPECT_FLOAT_EQ(f.at(0.15, 0.0), f.params.l_free);
}

// Two separate scans still accumulate: the once-per-cell rule is per insertion.
TEST(ScanInsertionTest, SeparateScansStillAccumulate) {
  InsertionFixture f;
  f.insert({{1.5, 0.0}});
  f.insert({{1.5, 0.0}});

  EXPECT_FLOAT_EQ(f.at(1.5, 0.0), 2 * f.params.l_occ);
  EXPECT_FLOAT_EQ(f.at(0.15, 0.0), 2 * f.params.l_free);
}

// The grid grows before ray casting, so a return past the initial bounds is not clipped.
TEST(ScanInsertionTest, ReturnBeyondTheInitialGridIsGrownIntoAndMarked) {
  InsertionFixture f;
  ASSERT_LT(f.grid.origin_x() + f.grid.width() * f.grid.resolution(), 5.0);

  f.insert({{5.0, 0.0}});

  EXPECT_GT(f.grid.width(), 40);
  EXPECT_FLOAT_EQ(f.at(5.0, 0.0), f.params.l_occ);
  EXPECT_FLOAT_EQ(f.at(2.5, 0.0), f.params.l_free);  // free space along the way
}

// Log-odds must saturate rather than run away over many scans.
TEST(ScanInsertionTest, RepeatedHitsClampAtTheConfiguredBound) {
  InsertionFixture f;
  for (int i = 0; i < 40; ++i) f.insert({{1.5, 0.0}});
  EXPECT_FLOAT_EQ(f.at(1.5, 0.0), f.params.clamp);
  EXPECT_FLOAT_EQ(f.at(0.15, 0.0), -f.params.clamp);
}

// --- at most two active submaps --------------------------------------------------

TEST(SubmapGraphTest, MakingRoomFinishesTheOldestAndKeepsAtMostTwoActive) {
  SubmapList graph;
  for (SubmapId id = 0; id < 2; ++id) {
    graph.active_submaps.push_back(std::make_shared<Submap>(id, IdentityPose(), 20, 20, 0.1));
  }
  ASSERT_EQ(graph.active_submaps.size(), 2U);
  ASSERT_TRUE(graph.history.empty());

  const auto finished = graph.make_room_for_new_submap(2);

  // The oldest was frozen and moved to history, leaving a slot for the new one.
  ASSERT_EQ(finished.size(), 1U);
  EXPECT_EQ(finished.front(), 0U);
  EXPECT_EQ(graph.active_submaps.size(), 1U);
  EXPECT_EQ(graph.active_submaps.front()->id(), 1U);
  ASSERT_EQ(graph.history.size(), 1U);
  EXPECT_EQ(graph.history.front()->id(), 0U);
  EXPECT_TRUE(graph.history.front()->is_finished());
  EXPECT_EQ(graph.history.front()->role(), SubmapRole::kAuthoritative);
}

// A submap short of its full insertion count is still finished: once it is no longer
// one of the two most recent it can never receive another scan.
TEST(SubmapGraphTest, MakingRoomFinishesAnUnderfilledSubmap) {
  SubmapList graph;
  auto old_submap = std::make_shared<Submap>(0, IdentityPose(), 20, 20, 0.1);
  old_submap->add_insertion();  // far short of 2 * submap_num_range_data
  graph.active_submaps.push_back(old_submap);
  graph.active_submaps.push_back(std::make_shared<Submap>(1, IdentityPose(), 20, 20, 0.1));

  EXPECT_EQ(graph.make_room_for_new_submap(2).size(), 1U);
  EXPECT_TRUE(old_submap->is_finished());
  EXPECT_EQ(old_submap->num_insertions(), 1);
}

TEST(SubmapGraphTest, MakingRoomDoesNothingBelowTheCap) {
  SubmapList graph;
  graph.active_submaps.push_back(std::make_shared<Submap>(0, IdentityPose(), 20, 20, 0.1));

  EXPECT_TRUE(graph.make_room_for_new_submap(2).empty());
  EXPECT_EQ(graph.active_submaps.size(), 1U);
  EXPECT_TRUE(graph.history.empty());
}

// Repeated creation must converge on the invariant, never drift above it.
TEST(SubmapGraphTest, RepeatedCreationNeverExceedsTwoActive) {
  SubmapList graph;
  for (SubmapId id = 0; id < 12; ++id) {
    graph.make_room_for_new_submap(2);
    graph.active_submaps.push_back(std::make_shared<Submap>(id, IdentityPose(), 20, 20, 0.1));
    EXPECT_LE(graph.active_submaps.size(), 2U);
  }
  EXPECT_EQ(graph.active_submaps.size(), 2U);
  EXPECT_EQ(graph.history.size(), 10U);
}

TEST(SubmapGraphTest, CountsInterConstraintsAndTrimsInactiveScanData) {
  SubmapList graph;
  auto active = std::make_shared<Submap>(1, IdentityPose(), 20, 20, 0.1);
  graph.active_submaps.push_back(active);

  auto active_data = std::make_shared<ScanNodeData>();
  active_data->returns.push_back({1.0, 0.0});
  auto old_data = std::make_shared<ScanNodeData>();
  old_data->returns.push_back({2.0, 0.0});
  graph.trajectory_nodes.push_back({3, active_data, IdentityPose()});
  graph.trajectory_nodes.push_back({4, old_data, IdentityPose()});

  graph.node_submap_constraints.push_back({
      1, 3, IdentityPose(), 1.0, 1.0, ConstraintTag::kIntraSubmap, 1.0, 1.0});
  graph.node_submap_constraints.push_back({
      0, 4, IdentityPose(), 1.0, 1.0, ConstraintTag::kInterSubmap, 0.8, 0.7});

  EXPECT_EQ(graph.inter_constraint_count(), 1U);
  graph.trim_scan_data_outside_active_submaps();
  EXPECT_NE(graph.trajectory_nodes[0].constant_data, nullptr);
  EXPECT_EQ(graph.trajectory_nodes[1].constant_data, nullptr);
}

}  // namespace
