// main.cpp - simulator_<ids> entry point. Parses argv then delegates to Simulator.

#include <Simulator/CommandLine.h>
#include <Simulator/Simulator.h>

int main(int argc, char** argv) {
    const auto cli = simulator::CommandLine::parse(argc, argv);
    if (!cli.has_value()) { return 1; }
    simulator::Simulator simulator;
    return simulator.run(*cli);
}
