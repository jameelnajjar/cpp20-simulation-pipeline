#pragma once

// NpyArray.h - 3D occupancy grid with .npy load/save.
// Lives in the Simulator because maps are a simulation artefact (Structuring PDF).
// Storage layout matches assignment 2: index = x*(ny*nz) + y*nz + z, int8 occupancy.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace simulator {

class NpyArray {
public:
    NpyArray() = default;
    NpyArray(std::size_t nx, std::size_t ny, std::size_t nz); // fills with Unmapped (-1)

    static NpyArray load(const std::filesystem::path& path);
    void save(const std::filesystem::path& path) const;

    [[nodiscard]] int8_t value(std::size_t x, std::size_t y, std::size_t z) const;
    void setRaw(std::size_t x, std::size_t y, std::size_t z, int8_t val);

    [[nodiscard]] std::size_t nx() const { return nx_; }
    [[nodiscard]] std::size_t ny() const { return ny_; }
    [[nodiscard]] std::size_t nz() const { return nz_; }
    [[nodiscard]] bool empty() const { return nx_ == 0 || ny_ == 0 || nz_ == 0; }

private:
    [[nodiscard]] std::size_t index(std::size_t x, std::size_t y, std::size_t z) const;

    std::size_t nx_ = 0;
    std::size_t ny_ = 0;
    std::size_t nz_ = 0;
    std::vector<int8_t> data_;
};

} // namespace simulator
