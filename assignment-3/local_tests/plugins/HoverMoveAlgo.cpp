#include "TestAlgoCommon.h"

namespace local_tests {
class HoverMoveAlgo final : public common::IMappingAlgorithm {
public:
    explicit HoverMoveAlgo(common::MappingAlgorithmDependencies d)
        : common::IMappingAlgorithm(std::move(d)) {}
    common::types::MappingStepCommand nextStep(const common::types::DroneState&,
                                               const common::types::LidarScanResult*) override {
        if (steps_ < 4) {
            ++steps_;
            return workingMove(MovementCommandType::Hover, 0.0);
        }
        return finished();
    }

private:
    int steps_ = 0;
};
} // namespace local_tests

using HoverMoveAlgo_local = local_tests::HoverMoveAlgo;
REGISTER_MAPPING_ALGORITHM(HoverMoveAlgo_local);
