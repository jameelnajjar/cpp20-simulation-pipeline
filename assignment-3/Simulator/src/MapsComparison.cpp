// MapsComparison.cpp - assignment 2 scoring, ported to simulator namespace.

#include <Simulator/MapsComparison.h>
#include <UserCommon/GeometryUtils.h>

#include <algorithm>
#include <cmath>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

namespace {

template <typename F>
void forEachVoxel(const common::types::MapConfig& cfg, F&& f) {
    const double res = ucm::cmOf(cfg.resolution);
    if (!(res > 0.0)) { return; }
    const double ox = ucm::cmOf(cfg.offset.x);
    const double oy = ucm::cmOf(cfg.offset.y);
    const double oz = ucm::cmOf(cfg.offset.z);
    const auto& b = cfg.boundaries;
    for (double x = ucm::cmOf(b.min_x) + res * 0.5; x < ucm::cmOf(b.max_x); x += res) {
        for (double y = ucm::cmOf(b.min_y) + res * 0.5; y < ucm::cmOf(b.max_y); y += res) {
            for (double z = ucm::cmOf(b.min_height) + res * 0.5; z < ucm::cmOf(b.max_height); z += res) {
                f(ucm::positionCm(ox + x, oy + y, oz + z));
            }
        }
    }
}

double compareOnePair(const common::IMap3D& origin, const common::IMap3D& target) {
    std::size_t total = 0;
    std::size_t correct = 0;
    std::size_t false_positives = 0;
    forEachVoxel(origin.getMapConfig(), [&](const common::Position3D& pos) {
        const auto orig = origin.atVoxel(pos);
        if (orig == common::types::VoxelOccupancy::OutOfBounds) { return; }
        if (!target.isInBounds(pos)) { return; }
        const auto tgt = target.atVoxel(pos);
        if (tgt == common::types::VoxelOccupancy::OutOfBounds) { return; }
        const bool orig_filled = (orig == common::types::VoxelOccupancy::Occupied);
        const bool tgt_filled = (tgt == common::types::VoxelOccupancy::Occupied);
        ++total;
        if (orig_filled == tgt_filled) {
            ++correct;
        } else if (tgt_filled && !orig_filled) {
            ++false_positives;
        }
    });
    if (total == 0) { return 100.0; }
    const double precision =
        static_cast<double>(correct) / static_cast<double>(total + false_positives);
    return std::clamp(precision * 100.0, 0.0, 100.0);
}

} // namespace

std::vector<double> MapsComparison::compare(
    const common::IMap3D& origin, const std::vector<const common::IMap3D*>& targets) {
    // HW2 scoring: Jaccard-style occupied-voxel precision, 0..100, -1 if a target is null.
    std::vector<double> scores;
    scores.reserve(targets.size());
    for (const common::IMap3D* target : targets) {
        scores.push_back(target ? compareOnePair(origin, *target) : -1.0);
    }
    return scores;
}

} // namespace simulator
