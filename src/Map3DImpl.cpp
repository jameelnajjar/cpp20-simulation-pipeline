#include <drone_mapper/Map3DImpl.h>

#include <cmath>
#include <stdexcept>

namespace drone_mapper {

namespace {
    double toCm(PhysicalLength v) { return v.numerical_value_in(cm); }
    double toCm(XLength v)        { return v.numerical_value_in(cm); }
    double toCm(YLength v)        { return v.numerical_value_in(cm); }
    double toCm(ZLength v)        { return v.numerical_value_in(cm); }
} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<TinyNPY::Array> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<TinyNPY::Array> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)), config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

bool Map3DImpl::worldToIndex(const Position3D& pos,
                              std::size_t& ix, std::size_t& iy, std::size_t& iz) const {
    if (map_->empty()) return false;
    const double res = toCm(config_.resolution);
    if (res <= 0.0) return false;

    const double ox = toCm(config_.offset.x);
    const double oy = toCm(config_.offset.y);
    const double oz = toCm(config_.offset.z);

    const double wx = toCm(pos.x);
    const double wy = toCm(pos.y);
    const double wz = toCm(pos.z);

    const double lx = wx - ox;
    const double ly = wy - oy;
    const double lz = wz - oz;

    if (lx < 0.0 || ly < 0.0 || lz < 0.0) return false;

    const auto ixi = static_cast<std::size_t>(std::floor(lx / res));
    const auto iyi = static_cast<std::size_t>(std::floor(ly / res));
    const auto izi = static_cast<std::size_t>(std::floor(lz / res));

    if (ixi >= map_->nx() || iyi >= map_->ny() || izi >= map_->nz()) return false;

    ix = ixi; iy = iyi; iz = izi;
    return true;
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    if (!isInBounds(pos)) return types::VoxelOccupancy::OutOfBounds;
    std::size_t ix = 0, iy = 0, iz = 0;
    if (!worldToIndex(pos, ix, iy, iz)) return types::VoxelOccupancy::OutOfBounds;
    const int8_t raw = map_->value(ix, iy, iz);
    switch (raw) {
        case -3: return types::VoxelOccupancy::PotentiallyOccupied;
        case -2: return types::VoxelOccupancy::OutOfBounds;
        case -1: return types::VoxelOccupancy::Unmapped;
        case  0: return types::VoxelOccupancy::Empty;
        case  1: return types::VoxelOccupancy::Occupied;
        default: return (raw > 0) ? types::VoxelOccupancy::Occupied : types::VoxelOccupancy::Unmapped;
    }
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const auto& b = config_.boundaries;
    const double wx = toCm(pos.x);
    const double wy = toCm(pos.y);
    const double wz = toCm(pos.z);
    return wx >= toCm(b.min_x) && wx <= toCm(b.max_x) &&
           wy >= toCm(b.min_y) && wy <= toCm(b.max_y) &&
           wz >= toCm(b.min_height) && wz <= toCm(b.max_height);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    std::size_t ix = 0, iy = 0, iz = 0;
    if (!worldToIndex(pos, ix, iy, iz)) return;
    int8_t raw = 0;
    switch (value) {
        case types::VoxelOccupancy::PotentiallyOccupied: raw = -3; break;
        case types::VoxelOccupancy::OutOfBounds:         raw = -2; break;
        case types::VoxelOccupancy::Unmapped:            raw = -1; break;
        case types::VoxelOccupancy::Empty:               raw =  0; break;
        case types::VoxelOccupancy::Occupied:            raw =  1; break;
    }
    map_->setRaw(ix, iy, iz, raw);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    map_->save(path);
}

} // namespace drone_mapper
