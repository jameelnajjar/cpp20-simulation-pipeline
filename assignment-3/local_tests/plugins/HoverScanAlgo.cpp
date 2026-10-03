#include "TestAlgoCommon.h"

namespace local_tests {
class HoverScanAlgo final : public common::IMappingAlgorithm {
public:
    explicit HoverScanAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        if (scans_ < 8) {
            ++scans_;
            return workingScan();
        }
        return finished();
    }

private:
    int scans_ = 0;
};
} // namespace local_tests

using HoverScanAlgo_local = local_tests::HoverScanAlgo;
REGISTER_MAPPING_ALGORITHM(HoverScanAlgo_local);
