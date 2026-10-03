#pragma once

// MapsComparison.h - Jaccard-style occupancy score from assignment 2.
// Simulator-owned: only the simulator has both the hidden map and the output map.

#include <Common/IMap3D.h>

#include <vector>

namespace simulator {

class MapsComparison {
public:
    [[nodiscard]] static std::vector<double> compare(const common::IMap3D& origin,
                                                     const std::vector<const common::IMap3D*>& targets);
};

} // namespace simulator
