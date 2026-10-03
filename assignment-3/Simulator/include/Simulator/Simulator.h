#pragma once

// Simulator.h - top-level orchestrator: CLI modes, lazy .so loading, thread pool.

#include <Simulator/CommandLine.h>

namespace simulator {

class Simulator {
public:
    [[nodiscard]] int run(const CommandLine& cli) const; // 0 on success, 1 after usage/IO errors
};

} // namespace simulator
