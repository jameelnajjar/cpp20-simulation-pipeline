// MappingAlgorithmRegistration.cpp - definition of the course-owned registration
// constructor. Compiled ONLY into the Simulator executable (assignment rule).

#include <Common/MappingAlgorithmRegistration.h>
#include <Simulator/Registrar.h>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    simulator::Registrar::instance().addAlgorithm(std::move(factory));
}

} // namespace common
