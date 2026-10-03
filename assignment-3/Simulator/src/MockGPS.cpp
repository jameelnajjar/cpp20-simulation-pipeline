// MockGPS.cpp - implements Simulator/MockGPS.h (assignment 2 MockGPS, new namespace).

#include <Simulator/MockGPS.h>

namespace simulator {

MockGPS::MockGPS(common::Position3D position, common::Orientation heading,
                 common::PhysicalLength resolution)
    : position_(position), heading_(heading), resolution_(resolution) {}

common::Position3D MockGPS::position() const { return position_; } // IGPS: current pose
common::Orientation MockGPS::heading() const { return heading_; }   // IGPS: current yaw/pitch
void MockGPS::setPosition(common::Position3D position) { position_ = position; } // used by MockMovement
void MockGPS::setHeading(common::Orientation heading) { heading_ = heading; }   // used by MockMovement
common::PhysicalLength MockGPS::resolution() const { return resolution_; } // GPS precision from mission YAML

} // namespace simulator
