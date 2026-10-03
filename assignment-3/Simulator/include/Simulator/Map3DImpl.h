#pragma once

// Map3DImpl.h - concrete common::IMutableMap3D used for both the hidden ground-truth
// map and the drone's output map. Simulator-owned (Structuring PDF).

#include <Common/IMutableMap3D.h>
#include <Simulator/NpyArray.h>

#include <memory>

namespace simulator {

class Map3DImpl final : public common::IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, common::types::MapConfig map_config);

    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const common::Position3D& pos) const override;
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override;
    void set(const common::Position3D& pos, common::types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

private:
    [[nodiscard]] bool worldToIndex(const common::Position3D& pos,
                                    std::size_t& ix, std::size_t& iy, std::size_t& iz) const;

    std::shared_ptr<NpyArray> map_;          // shared so MockLidar can keep reading while we own it
    common::types::MapConfig config_{};
};

} // namespace simulator
