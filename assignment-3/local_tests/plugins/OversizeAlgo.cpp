#include "TestAlgoCommon.h"

namespace local_tests {
class OversizeAlgo final : public common::IMappingAlgorithm {
public:
    explicit OversizeAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return workingMove(MovementCommandType::Advance, 400.0);
    }
};
} // namespace local_tests

using OversizeAlgo_local = local_tests::OversizeAlgo;
REGISTER_MAPPING_ALGORITHM(OversizeAlgo_local);
