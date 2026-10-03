#pragma once

// DroneControlImpl.h - our implementation of the course-owned mission_control::IDroneControl.
//
// This is the component that actually drives the aircraft: it asks the algorithm for the
// next command, sanity-checks it, drives the movement/LiDAR/GPS drivers, and folds every
// scan into the output map.
//
// It also implements the course's "Common issues and handling" table. Every row whose
// Detector column says "Drone controller" is handled here; the row numbers below refer to
// that document:
//   * faulty movement that would leave the mission bounds  -> amended, or ignored (rows 2, 10)
//   * command carrying invalid values                      -> retried, throws after N (row 3)
//   * empty "NOOP" command                                 -> retried, throws after N (row 4)
//   * empty LiDAR result                                   -> retried, throws after N (row 6)
//   * movement driver returning false                      -> retried, throws after N (row 7)
//   * movement larger than the configured maximum          -> split into several steps (row 8)
//   * GPS reporting out-of-bounds coordinates              -> ignored unless dead reckoning
//                                                             agrees, then throws (row 11)
//   * GPS reporting impossible coordinates after a move    -> re-read N times, then Error (row 12)

#include <MissionControl/IDroneControl.h> // course-owned interface implemented here

#include <Common/IDroneMovement.h>     // course-owned: movement driver interface
#include <Common/IGPS.h>               // course-owned: position/heading source
#include <Common/ILidar.h>             // course-owned: range sensor interface
#include <Common/IMappingAlgorithm.h>  // course-owned: the loaded algorithm we ask for commands
#include <Common/IMutableMap3D.h>      // course-owned: the map we fill in
#include <Common/types/DroneTypes.h>   // course-owned: commands, results, DroneState
#include <Common/types/MissionTypes.h> // course-owned: MissionConfigData / ErrorRef

#include <UserCommon/Logger.h> // our shared logger, injected by MissionControlImpl

#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

namespace mission_control_213309941_213727837 {

// Everything this class needs, bundled so the constructor stays readable.
// Composed by MissionControlImpl from the course-owned common::MissionControlDependencies.
struct DroneControlDependencies {
    common::types::DroneConfigData drone_config{};     // copied: max rotate/advance/elevate, radius
    common::types::MissionConfigData mission_config{}; // copied: bounds, gps resolution, max steps
    common::types::LidarConfigData lidar_config{};     // copied: z_min / z_max / d / fov_circles
    common::ILidar* lidar = nullptr;                   // borrowed from the Simulator
    common::IGPS* gps = nullptr;                       // borrowed from the Simulator
    common::IDroneMovement* movement = nullptr;        // borrowed from the Simulator
    common::IMutableMap3D* output_map = nullptr;       // borrowed from the Simulator
    common::IMappingAlgorithm* algorithm = nullptr;    // borrowed; lives in the Algorithm .so
    user_common_213309941_213727837::Logger* logger = nullptr; // borrowed from MissionControlImpl
};

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    // Takes the bundle by value and validates that no borrowed pointer is null.
    // Throws std::invalid_argument when a dependency is missing.
    explicit DroneControlImpl(DroneControlDependencies dependencies);

    // One simulation step, inherited from mission_control::IDroneControl.
    // Returns Continue, Completed (algorithm finished) or Error (soft failure).
    // Throws only for the unrecoverable cases the issues table asks us to throw for.
    [[nodiscard]] common::types::DroneStepResult step() override;

    // Current drone state, inherited from mission_control::IDroneControl.
    // Uses the GPS when its reading is trustworthy and dead reckoning otherwise.
    [[nodiscard]] common::types::DroneState state() const override;

    // Errors accumulated during the run; MissionControlImpl copies these into the
    // MissionRunResult it returns to the Simulator.
    [[nodiscard]] const std::vector<common::types::ErrorRef>& errors() const;

    // Number of movement commands that were split because they exceeded the drone maxima.
    [[nodiscard]] std::size_t splitCommandCount() const;

private:
    // One simulation step's worth of work. A single algorithm command becomes one
    // PendingAction when it is legal, or several when it has to be split (row 8);
    // the scan is attached to the last piece so it happens where the algorithm expected.
    struct PendingAction {
        std::optional<common::types::MovementCommand> movement{}; // what to fly, if anything
        std::optional<common::Orientation> scan{};                // where to look, if anywhere
    };

    // Runs one PendingAction: amend, drive, update the pose, then scan.
    [[nodiscard]] common::types::DroneStepResult executeAction(const PendingAction& action);

    // Turns one algorithm command into the queue of actions that will realise it.
    // Returns an empty queue when the command cannot be realised legally at all.
    [[nodiscard]] std::deque<PendingAction> planActions(
        const common::types::MappingStepCommand& command) const;

    // Validates that a command contains only finite, in-range numbers (row 3).
    [[nodiscard]] bool commandValuesAreValid(const common::types::MovementCommand& command) const;

    // Splits an over-large command into a queue of legal ones (row 8).
    // Returns the pieces in execution order; a legal command yields a single element,
    // and an empty result means "too large to realise, treat as invalid".
    [[nodiscard]] std::deque<common::types::MovementCommand> splitCommand(
        const common::types::MovementCommand& command) const;

    // True when a position lies inside mission_config.mission_bounds.
    [[nodiscard]] bool insideMissionBounds(const common::Position3D& position) const;

    // Predicts where `command` would leave the drone, starting from the dead-reckoned pose.
    [[nodiscard]] common::Position3D predictPosition(
        const common::types::MovementCommand& command) const;

    // Shortens an Advance/Elevate so the drone stays inside the mission bounds (row 10).
    // Returns std::nullopt when nothing legal remains, which means "ignore it" (row 2).
    [[nodiscard]] std::optional<common::types::MovementCommand> amendForBounds(
        const common::types::MovementCommand& command) const;

    // Applies a command to our dead-reckoning pose. Called after the driver accepts it.
    void applyToDeadReckoning(const common::types::MovementCommand& command);

    // Sends one already-validated command to the movement driver and handles a false
    // return value with bounded retries (row 7).
    [[nodiscard]] common::types::MovementResult driveMovement(
        const common::types::MovementCommand& command);

    // Fires the LiDAR, retrying on an empty result (row 6), then folds the scan into the map.
    void performScan(const common::Orientation& relative_direction);

    // Re-reads the GPS and reconciles it with dead reckoning (rows 11 and 12).
    // Returns false when the GPS could not be trusted after kMaxRetries attempts.
    [[nodiscard]] bool refreshPose();

    // Row 12: in-bounds GPS that jumped farther than 2*gps_resolution (or a large
    // heading jump) is treated as impossible and must not overwrite dead reckoning.
    [[nodiscard]] bool gpsAgreesWithDeadReckoning(const common::Position3D& gps_pos,
                                                  const common::Orientation& gps_heading) const;

    // Records an error both in errors() and in the injected log file.
    void recordError(const char* code, const std::string& message);

    DroneControlDependencies deps_;               // all injected collaborators
    common::Position3D dead_reckoned_position_{}; // pose integrated from accepted commands
    common::Orientation dead_reckoned_heading_{}; // heading integrated from accepted commands
    bool pose_initialised_ = false;               // seeded from the GPS on the first step()
    std::size_t step_index_ = 0;                  // steps executed so far, exposed via state()

    std::deque<PendingAction> pending_actions_{}; // remainder of a split command, scan kept on last piece
    std::optional<common::types::LidarScanResult> latest_scan_{};    // handed to the algorithm

    int invalid_command_strikes_ = 0; // consecutive invalid commands   (row 3)
    int noop_strikes_ = 0;            // consecutive empty commands     (row 4)
    int empty_scan_strikes_ = 0;      // consecutive empty scan results (row 6)
    int movement_failure_strikes_ = 0;// consecutive driver refusals    (row 7)
    std::size_t split_command_count_ = 0; // diagnostics for the verbose report

    std::vector<common::types::ErrorRef> errors_{}; // everything worth reporting upwards

    // "N tries" from the issues table. Three attempts is enough to ride out a transient
    // glitch while still failing fast on a genuinely broken component.
    static constexpr int kMaxRetries = 3;
    // Safety cap so a pathological oversize command cannot allocate millions of pieces.
    // Real missions (max_advance tens of cm, map tens of metres) stay far below this.
    static constexpr std::size_t kMaxSplitPieces = 8192;
};

} // namespace mission_control_213309941_213727837
