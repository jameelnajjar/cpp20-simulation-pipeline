#include <gtest/gtest.h>

#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>

#include <memory>

using namespace drone_mapper;

namespace {

// Stub run that always returns a given result.
class StubRun : public ISimulationRun {
public:
    explicit StubRun(types::SimulationResult result) : result_(std::move(result)) {}
    types::SimulationResult run() override { return result_; }
private:
    types::SimulationResult result_;
};

// Stub factory that creates StubRuns with configurable scores.
class StubFactory : public ISimulationRunFactory {
public:
    double score;
    int create_count = 0;
    bool throw_on_create = false;

    explicit StubFactory(double score = 90.0) : score(score) {}

    std::unique_ptr<ISimulationRun> create(const types::SimulationConfigData&,
                                            const types::MissionConfigData&,
                                            const types::DroneConfigData&,
                                            const types::LidarConfigData&,
                                            const std::filesystem::path&) override {
        ++create_count;
        if (throw_on_create) throw std::runtime_error("factory error");
        types::SimulationResult r;
        r.mission_score = score;
        r.mission_results.push_back({types::MissionRunStatus::Completed, 10, {}});
        return std::make_unique<StubRun>(r);
    }
};

types::SimulationCompositionData makeComposition(int sim_count = 1, int mission_count = 1,
                                                  int drone_count = 1, int lidar_count = 1) {
    types::SimulationCompositionData comp;
    comp.composition_file = "test_composition.yaml";

    for (int s = 0; s < sim_count; ++s) {
        std::vector<types::MissionConfigData> missions;
        for (int m = 0; m < mission_count; ++m) {
            missions.push_back({});
        }
        comp.simulation_mission_groups.emplace_back(types::SimulationConfigData{}, std::move(missions));
    }
    for (int d = 0; d < drone_count; ++d) {
        comp.drones.push_back({});
    }
    for (int l = 0; l < lidar_count; ++l) {
        comp.lidars.push_back({});
    }
    return comp;
}

} // namespace

// Constructor throws on null factory.
TEST(SimulationManager, ConstructorThrowsOnNull) {
    EXPECT_THROW(SimulationManager{nullptr}, std::invalid_argument);
}

// Run returns correct number of results (cartesian product).
TEST(SimulationManager, RunCountMatchesCartesianProduct) {
    auto factory = std::make_unique<StubFactory>();
    StubFactory* raw = factory.get();
    SimulationManager manager{std::move(factory)};

    const auto comp = makeComposition(2, 2, 2, 2);
    const auto report = manager.run(comp, std::filesystem::temp_directory_path());

    // 2*2*2*2 = 16 runs
    EXPECT_EQ(report.runs.size(), 16u);
    EXPECT_EQ(raw->create_count, 16);
}

// Score is propagated from runs.
TEST(SimulationManager, ReportContainsScores) {
    auto factory = std::make_unique<StubFactory>(75.5);
    SimulationManager manager{std::move(factory)};

    const auto comp = makeComposition();
    const auto report = manager.run(comp, std::filesystem::temp_directory_path());

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_NEAR(report.runs[0].mission_score, 75.5, 0.01);
}

// Factory error produces -1 score and continues to other runs.
TEST(SimulationManager, FactoryErrorContinuesToNextRun) {
    auto factory = std::make_unique<StubFactory>(80.0);
    factory->throw_on_create = true;
    SimulationManager manager{std::move(factory)};

    const auto comp = makeComposition(2);
    const auto report = manager.run(comp, std::filesystem::temp_directory_path());

    ASSERT_EQ(report.runs.size(), 2u);
    for (const auto& r : report.runs) {
        EXPECT_NEAR(r.mission_score, -1.0, 0.001);
    }
}

// Report has metric and generated_at_utc set.
TEST(SimulationManager, ReportHasMetadata) {
    auto factory = std::make_unique<StubFactory>();
    SimulationManager manager{std::move(factory)};

    const auto comp = makeComposition();
    const auto report = manager.run(comp, std::filesystem::temp_directory_path());

    EXPECT_FALSE(report.metric.empty());
    EXPECT_FALSE(report.generated_at_utc.empty());
}

// Empty composition produces empty report.
TEST(SimulationManager, EmptyCompositionProducesEmptyReport) {
    auto factory = std::make_unique<StubFactory>();
    SimulationManager manager{std::move(factory)};

    types::SimulationCompositionData empty_comp;
    const auto report = manager.run(empty_comp, std::filesystem::temp_directory_path());

    EXPECT_EQ(report.runs.size(), 0u);
}
