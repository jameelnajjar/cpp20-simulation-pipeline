#include "TestAlgoCommon.h"

namespace local_tests {
class InvalidAlgo final : public common::IMappingAlgorithm {
public:
    explicit InvalidAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return workingMove(MovementCommandType::Advance, 10.0, -45.0);
    }
};
} // namespace local_tests

using InvalidAlgo_local = local_tests::InvalidAlgo;
REGISTER_MAPPING_ALGORITHM(InvalidAlgo_local);
