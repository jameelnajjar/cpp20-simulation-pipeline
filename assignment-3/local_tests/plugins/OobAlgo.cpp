#include "TestAlgoCommon.h"

namespace local_tests {
class OobAlgo final : public common::IMappingAlgorithm {
public:
    explicit OobAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return workingMove(MovementCommandType::Advance, 5000.0);
    }
};
} // namespace local_tests

using OobAlgo_local = local_tests::OobAlgo;
REGISTER_MAPPING_ALGORITHM(OobAlgo_local);
