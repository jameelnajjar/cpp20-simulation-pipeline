#include "TestAlgoCommon.h"

namespace local_tests {
class ElevateFarAlgo final : public common::IMappingAlgorithm {
public:
    explicit ElevateFarAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return workingMove(MovementCommandType::Elevate, 200.0);
    }
};
} // namespace local_tests

using ElevateFarAlgo_local = local_tests::ElevateFarAlgo;
REGISTER_MAPPING_ALGORITHM(ElevateFarAlgo_local);
