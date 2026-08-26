#include <drone_mapper/NpyArray.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace drone_mapper {

namespace {

// Parse "{'descr': ..., 'fortran_order': False, 'shape': (X, Y, Z), }"
std::tuple<std::size_t, std::size_t, std::size_t> parseNpyHeader(const std::string& header) {
    std::regex shape_re(R"(\((\d+),\s*(\d+),\s*(\d+)\))");
    std::smatch m;
    if (!std::regex_search(header, m, shape_re)) {
        throw std::runtime_error("NpyArray: could not parse shape from NPY header: " + header);
    }
    return {std::stoull(m[1]), std::stoull(m[2]), std::stoull(m[3])};
}

} // namespace

// Default -1 = Unmapped for all cells.
NpyArray::NpyArray(std::size_t nx, std::size_t ny, std::size_t nz)
    : nx_(nx), ny_(ny), nz_(nz), data_(nx * ny * nz, static_cast<int8_t>(-1)) {}

std::size_t NpyArray::index(std::size_t x, std::size_t y, std::size_t z) const {
    return x * (ny_ * nz_) + y * nz_ + z;
}

bool NpyArray::at(std::size_t x, std::size_t y, std::size_t z) const {
    return data_[index(x, y, z)] == static_cast<int8_t>(1);
}

int8_t NpyArray::value(std::size_t x, std::size_t y, std::size_t z) const {
    return data_[index(x, y, z)];
}

void NpyArray::set(std::size_t x, std::size_t y, std::size_t z, bool occupied) {
    data_[index(x, y, z)] = occupied ? static_cast<int8_t>(1) : static_cast<int8_t>(0);
}

void NpyArray::setRaw(std::size_t x, std::size_t y, std::size_t z, int8_t val) {
    data_[index(x, y, z)] = val;
}

NpyArray NpyArray::load(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::runtime_error("NpyArray: cannot open file: " + path.string());
    }

    // Magic + version
    char magic[6];
    f.read(magic, 6);
    if (f.gcount() < 6 || std::memcmp(magic, "\x93NUMPY", 6) != 0) {
        throw std::runtime_error("NpyArray: not a valid NPY file: " + path.string());
    }
    uint8_t ver_major = 0, ver_minor = 0;
    f.read(reinterpret_cast<char*>(&ver_major), 1);
    f.read(reinterpret_cast<char*>(&ver_minor), 1);

    // Header length
    uint32_t header_len = 0;
    if (ver_major == 1) {
        uint16_t hlen16 = 0;
        f.read(reinterpret_cast<char*>(&hlen16), 2);
        header_len = hlen16;
    } else {
        f.read(reinterpret_cast<char*>(&header_len), 4);
    }

    std::string header(header_len, '\0');
    f.read(header.data(), static_cast<std::streamsize>(header_len));

    auto [nx, ny, nz] = parseNpyHeader(header);

    NpyArray arr(nx, ny, nz);
    const std::size_t total = nx * ny * nz;
    // Read raw bytes into temp buffer; values may be 0/1 bool or int8_t
    std::vector<int8_t> buf(total);
    f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(total));
    if (static_cast<std::size_t>(f.gcount()) < total) {
        throw std::runtime_error("NpyArray: unexpected EOF reading data: " + path.string());
    }
    // Copy into arr, mapping: any non-zero → 1 (Occupied), 0 → 0 (Empty)
    // We don't preserve -1 (Unmapped) from files; input maps are binary 0/1.
    for (std::size_t i = 0; i < total; ++i) {
        arr.data_[i] = (buf[i] != 0) ? static_cast<int8_t>(1) : static_cast<int8_t>(0);
    }
    return arr;
}

void NpyArray::save(const std::filesystem::path& path) const {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) {
        throw std::runtime_error("NpyArray: cannot create file: " + path.string());
    }

    // Build header string with int8 dtype
    std::ostringstream hss;
    hss << "{'descr': '|i1', 'fortran_order': False, 'shape': ("
        << nx_ << ", " << ny_ << ", " << nz_ << "), }";
    std::string hdr_body = hss.str();

    // Pad to 64-byte alignment (version 1 requires (10 + header_len) % 64 == 0)
    // 10 = magic(6) + major(1) + minor(1) + len(2)
    std::size_t total_prefix = 10 + hdr_body.size() + 1; // +1 for newline
    std::size_t pad = (64 - (total_prefix % 64)) % 64;
    hdr_body += std::string(pad, ' ');
    hdr_body += '\n';

    const uint16_t hlen = static_cast<uint16_t>(hdr_body.size());

    f.write("\x93NUMPY", 6);
    const uint8_t ver_major = 1, ver_minor = 0;
    f.write(reinterpret_cast<const char*>(&ver_major), 1);
    f.write(reinterpret_cast<const char*>(&ver_minor), 1);
    f.write(reinterpret_cast<const char*>(&hlen), 2);
    f.write(hdr_body.c_str(), static_cast<std::streamsize>(hdr_body.size()));
    f.write(reinterpret_cast<const char*>(data_.data()), static_cast<std::streamsize>(data_.size()));
}

} // namespace drone_mapper
