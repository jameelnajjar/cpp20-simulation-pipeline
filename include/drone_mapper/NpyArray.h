#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <vector>

namespace drone_mapper {

// 3D voxel array backed by NPY format.
// Each cell stores an int8_t value representing VoxelOccupancy:
//   -1 = Unmapped (default), 0 = Empty, 1 = Occupied, etc.
// Index order: (x, y, z), row-major: index = x*(ny*nz) + y*nz + z
class NpyArray {
public:
    NpyArray() = default;
    NpyArray(std::size_t nx, std::size_t ny, std::size_t nz);

    // Copy constructor (used in tests)
    NpyArray(const NpyArray&) = default;
    NpyArray& operator=(const NpyArray&) = default;
    NpyArray(NpyArray&&) = default;
    NpyArray& operator=(NpyArray&&) = default;

    // Load from .npy file. Throws on error.
    static NpyArray load(const std::filesystem::path& path);
    // Save to .npy file. Throws on error.
    void save(const std::filesystem::path& path) const;

    // Returns true if the voxel has an Occupied value (1).
    [[nodiscard]] bool at(std::size_t x, std::size_t y, std::size_t z) const;
    // Returns the raw int8_t occupancy value.
    [[nodiscard]] int8_t value(std::size_t x, std::size_t y, std::size_t z) const;
    // Set voxel to occupied (true=1) or empty (false=0).
    void set(std::size_t x, std::size_t y, std::size_t z, bool occupied);
    // Set voxel to an explicit int8_t value.
    void setRaw(std::size_t x, std::size_t y, std::size_t z, int8_t val);

    [[nodiscard]] std::size_t nx() const { return nx_; }
    [[nodiscard]] std::size_t ny() const { return ny_; }
    [[nodiscard]] std::size_t nz() const { return nz_; }
    [[nodiscard]] bool empty() const { return nx_ == 0 || ny_ == 0 || nz_ == 0; }

private:
    std::size_t nx_ = 0;
    std::size_t ny_ = 0;
    std::size_t nz_ = 0;
    // Stores int8_t values; default -1 = Unmapped.
    std::vector<int8_t> data_;

    [[nodiscard]] std::size_t index(std::size_t x, std::size_t y, std::size_t z) const;
};

} // namespace drone_mapper
