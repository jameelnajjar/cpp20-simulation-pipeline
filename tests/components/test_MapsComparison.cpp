#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/NpyArray.h>

using namespace drone_mapper;

namespace {

types::MapConfig makeConfig(double res_cm = 10.0, double size_cm = 100.0) {
    types::MapConfig cfg;
    cfg.resolution = res_cm * cm;
    cfg.offset = Position3D{};
    cfg.boundaries = types::MappingBounds{
        0.0 * x_extent[cm], size_cm * x_extent[cm],
        0.0 * y_extent[cm], size_cm * y_extent[cm],
        0.0 * z_extent[cm], size_cm * z_extent[cm],
    };
    return cfg;
}

} // namespace

// Two identical maps should score 100.
TEST(MapsComparison, IdenticalMapsScore100) {
    constexpr std::size_t N = 5;
    auto origin_arr = std::make_shared<NpyArray>(N, N, N);
    origin_arr->set(1, 1, 1, true);
    origin_arr->set(2, 2, 2, true);

    auto target_arr = std::make_shared<NpyArray>(*origin_arr);

    Map3DImpl origin(origin_arr, makeConfig());
    Map3DImpl target(target_arr, makeConfig());

    std::vector<IMap3D*> targets = {&target};
    const auto scores = MapsComparison::compare(origin, targets);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 100.0, 0.01);
}

// Two empty maps should also score 100 (nothing to compare).
TEST(MapsComparison, EmptyMapsScore100) {
    constexpr std::size_t N = 5;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    auto tgt  = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl origin(orig, makeConfig());
    Map3DImpl target(tgt,  makeConfig());
    std::vector<IMap3D*> targets = {&target};
    const auto scores = MapsComparison::compare(origin, targets);
    EXPECT_NEAR(scores[0], 100.0, 0.01);
}

// Target with all voxels occupied vs. empty origin → many false positives → low score.
TEST(MapsComparison, VeryDistinctMapsScoreNear0) {
    constexpr std::size_t N = 5;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    auto tgt  = std::make_shared<NpyArray>(N, N, N);
    for (std::size_t x = 0; x < N; ++x)
        for (std::size_t y = 0; y < N; ++y)
            for (std::size_t z = 0; z < N; ++z)
                tgt->set(x, y, z, true);

    Map3DImpl origin(orig, makeConfig());
    Map3DImpl target(tgt,  makeConfig());
    std::vector<IMap3D*> targets = {&target};
    const auto scores = MapsComparison::compare(origin, targets);
    EXPECT_LT(scores[0], 50.0);
}

// Near-identical maps (one voxel different) should score close to but not 100.
TEST(MapsComparison, NearlyIdenticalMapsScoreNear100) {
    constexpr std::size_t N = 10;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    for (std::size_t x = 0; x < N; ++x)
        for (std::size_t y = 0; y < N; ++y)
            for (std::size_t z = 0; z < N; ++z)
                orig->set(x, y, z, (x + y + z) % 3 == 0);

    auto tgt = std::make_shared<NpyArray>(*orig);
    tgt->set(0, 0, 0, !orig->at(0, 0, 0)); // flip one voxel

    Map3DImpl origin(orig, makeConfig(10.0, 100.0));
    Map3DImpl target(tgt,  makeConfig(10.0, 100.0));
    std::vector<IMap3D*> targets = {&target};
    const auto scores = MapsComparison::compare(origin, targets);
    EXPECT_GT(scores[0], 50.0);
    EXPECT_LE(scores[0], 100.0);
}

// Null pointer in targets returns -1.
TEST(MapsComparison, NullTargetReturnsMinusOne) {
    constexpr std::size_t N = 3;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl origin(orig, makeConfig());
    std::vector<IMap3D*> targets = {nullptr};
    const auto scores = MapsComparison::compare(origin, targets);
    EXPECT_DOUBLE_EQ(scores[0], -1.0);
}

// Multiple targets - vector size matches.
TEST(MapsComparison, MultipleTargetsReturnMultipleScores) {
    constexpr std::size_t N = 3;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    auto t1   = std::make_shared<NpyArray>(N, N, N);
    auto t2   = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl origin(orig, makeConfig());
    Map3DImpl target1(t1, makeConfig());
    Map3DImpl target2(t2, makeConfig());
    std::vector<IMap3D*> targets = {&target1, &target2};
    const auto scores = MapsComparison::compare(origin, targets);
    EXPECT_EQ(scores.size(), 2u);
}

// Origin with occupied voxels vs target that misses them all → score < 100.
// This catches any bug that blindly returns 100 for all inputs.
TEST(MapsComparison, MissingOccupiedVoxelsReduceScore) {
    constexpr std::size_t N = 5;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    orig->set(1, 1, 1, true);
    orig->set(2, 2, 2, true);
    orig->set(3, 3, 3, true);

    auto tgt = std::make_shared<NpyArray>(N, N, N);
    // Target has no occupied voxels — all false negatives

    Map3DImpl origin(orig, makeConfig());
    Map3DImpl target(tgt, makeConfig());
    std::vector<IMap3D*> targets = {&target};
    const auto scores = MapsComparison::compare(origin, targets);
    EXPECT_LT(scores[0], 100.0);
}

// Target with false positives (extra occupied) reduces score below 100.
TEST(MapsComparison, FalsePositivesPenalizeScore) {
    constexpr std::size_t N = 5;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    // Origin: all empty

    auto tgt = std::make_shared<NpyArray>(N, N, N);
    for (std::size_t x = 0; x < N; ++x)
        for (std::size_t y = 0; y < N; ++y)
            for (std::size_t z = 0; z < N; ++z)
                tgt->set(x, y, z, true);  // target is all Occupied

    Map3DImpl origin(orig, makeConfig());
    Map3DImpl target(tgt, makeConfig());
    std::vector<IMap3D*> targets = {&target};
    const auto scores = MapsComparison::compare(origin, targets);
    EXPECT_LT(scores[0], 100.0);
}

// Scores for both identical and different targets in the same batch are correct.
TEST(MapsComparison, BatchIdenticalAndDifferentScores) {
    constexpr std::size_t N = 5;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    orig->set(1, 1, 1, true);

    auto same = std::make_shared<NpyArray>(*orig);    // identical copy
    auto diff = std::make_shared<NpyArray>(N, N, N);  // completely empty

    Map3DImpl origin(orig, makeConfig());
    Map3DImpl identical(same, makeConfig());
    Map3DImpl different(diff, makeConfig());

    std::vector<IMap3D*> targets = {&identical, &different};
    const auto scores = MapsComparison::compare(origin, targets);

    ASSERT_EQ(scores.size(), 2u);
    EXPECT_NEAR(scores[0], 100.0, 0.01);  // identical → 100
    EXPECT_LT(scores[1], 100.0);           // different → < 100
}

// All returned scores are in the valid [0, 100] range.
TEST(MapsComparison, ScoresAreInValidRange) {
    constexpr std::size_t N = 5;
    auto orig = std::make_shared<NpyArray>(N, N, N);
    orig->set(2, 2, 2, true);

    auto t1 = std::make_shared<NpyArray>(*orig);
    auto t2 = std::make_shared<NpyArray>(N, N, N);

    Map3DImpl origin(orig, makeConfig());
    Map3DImpl target1(t1, makeConfig());
    Map3DImpl target2(t2, makeConfig());

    std::vector<IMap3D*> targets = {&target1, &target2};
    const auto scores = MapsComparison::compare(origin, targets);

    for (const double s : scores) {
        EXPECT_GE(s, 0.0);
        EXPECT_LE(s, 100.0);
    }
}
