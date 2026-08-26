#include <drone_mapper/MapsComparison.h>

#include <cmath>

namespace drone_mapper {

namespace {

double toCm(PhysicalLength v)  { return v.numerical_value_in(cm); }
double toCmX(XLength v)        { return v.numerical_value_in(cm); }
double toCmY(YLength v)        { return v.numerical_value_in(cm); }
double toCmZ(ZLength v)        { return v.numerical_value_in(cm); }

// Iterate over all voxel positions in the map defined by config.
// Calls f(Position3D) for each voxel center.
template<typename F>
void forEachVoxel(const types::MapConfig& cfg, F&& f) {
    const double res = toCm(cfg.resolution);
    if (res <= 0.0) return;
    const double ox = toCmX(cfg.offset.x);
    const double oy = toCmY(cfg.offset.y);
    const double oz = toCmZ(cfg.offset.z);
    const double min_x = toCmX(cfg.boundaries.min_x);
    const double max_x = toCmX(cfg.boundaries.max_x);
    const double min_y = toCmY(cfg.boundaries.min_y);
    const double max_y = toCmY(cfg.boundaries.max_y);
    const double min_z = toCmZ(cfg.boundaries.min_height);
    const double max_z = toCmZ(cfg.boundaries.max_height);

    for (double x = min_x + res * 0.5; x < max_x; x += res) {
        for (double y = min_y + res * 0.5; y < max_y; y += res) {
            for (double z = min_z + res * 0.5; z < max_z; z += res) {
                const Position3D pos{
                    (ox + x) * x_extent[cm],
                    (oy + y) * y_extent[cm],
                    (oz + z) * z_extent[cm],
                };
                f(pos);
            }
        }
    }
}

double compareOnePair(const IMap3D& origin, const IMap3D& target) {
    // Score: Jaccard-style measure on occupied voxels.
    // Unmapped voxels are treated as Empty (not occupied) for comparison purposes.
    // Both empty maps → 100; fully-wrong target → near 0.
    std::size_t total = 0;
    std::size_t correct = 0;
    std::size_t false_positives = 0;

    const types::MapConfig& oCfg = origin.getMapConfig();
    const types::MapConfig& tCfg = target.getMapConfig();
    (void)tCfg;

    forEachVoxel(oCfg, [&](const Position3D& pos) {
        const auto orig_occ = origin.atVoxel(pos);
        // Skip voxels that fall outside the actual array storage.
        if (orig_occ == types::VoxelOccupancy::OutOfBounds) return;
        if (!target.isInBounds(pos)) return;
        const auto tgt_occ = target.atVoxel(pos);
        if (tgt_occ == types::VoxelOccupancy::OutOfBounds) return;

        // Treat Unmapped and PotentiallyOccupied as "not occupied" for scoring.
        const bool orig_filled = (orig_occ == types::VoxelOccupancy::Occupied);
        const bool tgt_filled  = (tgt_occ  == types::VoxelOccupancy::Occupied);

        ++total;
        if (orig_filled == tgt_filled) {
            ++correct;
        } else if (tgt_filled && !orig_filled) {
            ++false_positives;
        }
    });

    if (total == 0) return 100.0;
    const double precision = static_cast<double>(correct) / static_cast<double>(total + false_positives);
    return std::clamp(precision * 100.0, 0.0, 100.0);
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                              const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());
    for (const IMap3D* target : targets) {
        if (!target) {
            scores.push_back(-1.0);
        } else {
            scores.push_back(compareOnePair(origin, *target));
        }
    }
    return scores;
}

} // namespace drone_mapper
