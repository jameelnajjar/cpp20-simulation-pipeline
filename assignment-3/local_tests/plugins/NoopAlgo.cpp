#include "TestAlgoCommon.h"

namespace local_tests {
class NoopAlgo final : public common::IMappingAlgorithm {
public:
    explicit NoopAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        return noop();
    }
};
} // namespace local_tests

using NoopAlgo_local = local_tests::NoopAlgo;
REGISTER_MAPPING_ALGORITHM(NoopAlgo_local);
