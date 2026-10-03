#include "TestAlgoCommon.h"

namespace local_tests {
class WallAlgo final : public common::IMappingAlgorithm {
public:
    explicit WallAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        // Advance along +X into the wall placed at x=50cm on the test map.
        return workingMove(MovementCommandType::Advance, 25.0);
    }
};
} // namespace local_tests

using WallAlgo_local = local_tests::WallAlgo;
REGISTER_MAPPING_ALGORITHM(WallAlgo_local);
