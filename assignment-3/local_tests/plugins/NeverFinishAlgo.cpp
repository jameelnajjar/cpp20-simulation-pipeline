#include "TestAlgoCommon.h"

namespace local_tests {
class NeverFinishAlgo final : public common::IMappingAlgorithm {
public:
    explicit NeverFinishAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return workingScan();
    }
};
} // namespace local_tests

using NeverFinishAlgo_local = local_tests::NeverFinishAlgo;
REGISTER_MAPPING_ALGORITHM(NeverFinishAlgo_local);
