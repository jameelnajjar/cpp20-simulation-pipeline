// NpyArray.cpp - NumPy .npy v1 reader/writer. Inherited from assignment 2 NpyArray.

#include <Simulator/NpyArray.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace simulator {

NpyArray::NpyArray(std::size_t nx, std::size_t ny, std::size_t nz)
    : nx_(nx), ny_(ny), nz_(nz), data_(nx * ny * nz, static_cast<int8_t>(-1)) {}

std::size_t NpyArray::index(std::size_t x, std::size_t y, std::size_t z) const {
    return x * (ny_ * nz_) + y * nz_ + z;
}

int8_t NpyArray::value(std::size_t x, std::size_t y, std::size_t z) const {
    return data_[index(x, y, z)];
}

void NpyArray::setRaw(std::size_t x, std::size_t y, std::size_t z, int8_t val) {
    data_[index(x, y, z)] = val;
}

NpyArray NpyArray::load(const std::filesystem::path& path) { // NumPy v1/v2 .npy reader
    std::ifstream f(path, std::ios::binary);
    if (!f) { throw std::runtime_error("NpyArray: cannot open " + path.string()); }

    char magic[6];
    f.read(magic, 6);
    if (f.gcount() < 6 || std::memcmp(magic, "\x93NUMPY", 6) != 0) {
        throw std::runtime_error("NpyArray: not a valid NPY file: " + path.string());
    }
    uint8_t ver_major = 0, ver_minor = 0;
    f.read(reinterpret_cast<char*>(&ver_major), 1);
    f.read(reinterpret_cast<char*>(&ver_minor), 1);

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

    std::regex shape_re(R"(\((\d+),\s*(\d+),\s*(\d+)\))");
    std::smatch match;
    if (!std::regex_search(header, match, shape_re)) {
        throw std::runtime_error("NpyArray: could not parse shape: " + header);
    }
    const std::size_t nx = static_cast<std::size_t>(std::stoull(match[1]));
    const std::size_t ny = static_cast<std::size_t>(std::stoull(match[2]));
    const std::size_t nz = static_cast<std::size_t>(std::stoull(match[3]));

    NpyArray arr(nx, ny, nz);
    const std::size_t total = nx * ny * nz;
    std::vector<int8_t> buf(total);
    f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(total));
    if (static_cast<std::size_t>(f.gcount()) < total) {
        throw std::runtime_error("NpyArray: unexpected EOF: " + path.string());
    }
    for (std::size_t i = 0; i < total; ++i) {
        arr.data_[i] = (buf[i] != 0) ? static_cast<int8_t>(1) : static_cast<int8_t>(0);
    }
    return arr;
}

void NpyArray::save(const std::filesystem::path& path) const { // NumPy v1 |i1 writer
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) { throw std::runtime_error("NpyArray: cannot create " + path.string()); }

    std::ostringstream hss;
    hss << "{'descr': '|i1', 'fortran_order': False, 'shape': ("
        << nx_ << ", " << ny_ << ", " << nz_ << "), }";
    std::string hdr_body = hss.str();
    const std::size_t total_prefix = 10 + hdr_body.size() + 1;
    const std::size_t pad = (64 - (total_prefix % 64)) % 64;
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

} // namespace simulator
