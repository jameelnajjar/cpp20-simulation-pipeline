// Map3DImpl.cpp - world-to-index occupancy grid. Ported from assignment 2 Map3DImpl.

#include <Simulator/Map3DImpl.h>
#include <UserCommon/GeometryUtils.h>

#include <cmath>
#include <stdexcept>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, common::types::MapConfig map_config)
    : map_(std::move(map_ptr)), config_(map_config) {
    if (!map_) { throw std::invalid_argument("Map3DImpl requires a valid map pointer."); }
}

bool Map3DImpl::worldToIndex(const common::Position3D& pos,
                             std::size_t& ix, std::size_t& iy, std::size_t& iz) const {
    if (map_->empty()) { return false; }
    const double res = ucm::cmOf(config_.resolution);
    if (!(res > 0.0)) { return false; }

    const double lx = ucm::cmOf(pos.x) - ucm::cmOf(config_.offset.x);
    const double ly = ucm::cmOf(pos.y) - ucm::cmOf(config_.offset.y);
    const double lz = ucm::cmOf(pos.z) - ucm::cmOf(config_.offset.z);
    if (lx < 0.0 || ly < 0.0 || lz < 0.0) { return false; }

    const auto ixi = static_cast<std::size_t>(std::floor(lx / res));
    const auto iyi = static_cast<std::size_t>(std::floor(ly / res));
    const auto izi = static_cast<std::size_t>(std::floor(lz / res));
    if (ixi >= map_->nx() || iyi >= map_->ny() || izi >= map_->nz()) { return false; }
    ix = ixi; iy = iyi; iz = izi;
    return true;
}

common::types::VoxelOccupancy Map3DImpl::atVoxel(const common::Position3D& pos) const {
    if (!isInBounds(pos)) { return common::types::VoxelOccupancy::OutOfBounds; }
    std::size_t ix = 0, iy = 0, iz = 0;
    if (!worldToIndex(pos, ix, iy, iz)) { return common::types::VoxelOccupancy::OutOfBounds; }
    switch (map_->value(ix, iy, iz)) {
        case -3: return common::types::VoxelOccupancy::PotentiallyOccupied;
        case -2: return common::types::VoxelOccupancy::OutOfBounds;
        case -1: return common::types::VoxelOccupancy::Unmapped;
        case  0: return common::types::VoxelOccupancy::Empty;
        case  1: return common::types::VoxelOccupancy::Occupied;
        default: return (map_->value(ix, iy, iz) > 0)
                            ? common::types::VoxelOccupancy::Occupied
                            : common::types::VoxelOccupancy::Unmapped;
    }
}

common::types::MapConfig Map3DImpl::getMapConfig() const { return config_; } // IMap3D

void Map3DImpl::save(const std::filesystem::path& path) const { map_->save(path); } // IMutableMap3D

bool Map3DImpl::isInBounds(const common::Position3D& pos) const { // IMap3D, uses MappingBounds
    return ucm::isInsideBounds(config_.boundaries, pos);
}

void Map3DImpl::set(const common::Position3D& pos, common::types::VoxelOccupancy value) { // IMutableMap3D
    std::size_t ix = 0, iy = 0, iz = 0;
    if (!worldToIndex(pos, ix, iy, iz)) { return; }
    int8_t raw = 0;
    switch (value) {
        case common::types::VoxelOccupancy::PotentiallyOccupied: raw = -3; break;
        case common::types::VoxelOccupancy::OutOfBounds:         raw = -2; break;
        case common::types::VoxelOccupancy::Unmapped:            raw = -1; break;
        case common::types::VoxelOccupancy::Empty:               raw =  0; break;
        case common::types::VoxelOccupancy::Occupied:            raw =  1; break;
    }
    map_->setRaw(ix, iy, iz, raw);
}

} // namespace simulator
