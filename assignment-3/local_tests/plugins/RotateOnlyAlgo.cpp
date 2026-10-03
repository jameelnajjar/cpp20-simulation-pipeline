#include "TestAlgoCommon.h"

namespace local_tests {
class RotateOnlyAlgo final : public common::IMappingAlgorithm {
public:
    explicit RotateOnlyAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return workingMove(MovementCommandType::Rotate, 0.0, 45.0);
    }
};
} // namespace local_tests

using RotateOnlyAlgo_local = local_tests::RotateOnlyAlgo;
REGISTER_MAPPING_ALGORITHM(RotateOnlyAlgo_local);
