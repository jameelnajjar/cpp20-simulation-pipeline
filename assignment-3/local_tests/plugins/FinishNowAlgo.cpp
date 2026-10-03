#include "TestAlgoCommon.h"

namespace local_tests {
class FinishNowAlgo final : public common::IMappingAlgorithm {
public:
    explicit FinishNowAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return finished();
    }
};
} // namespace local_tests

using FinishNowAlgo_local = local_tests::FinishNowAlgo;
REGISTER_MAPPING_ALGORITHM(FinishNowAlgo_local);
