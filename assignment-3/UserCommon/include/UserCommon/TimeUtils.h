#pragma once

// TimeUtils.h - UTC formatting and collision-free name generation.
// Shared by the Simulator (output folder names "comparative_results_<time>",
// the generated_at_utc report field, per-run map file names) and by the
// MissionControl (verbose file names and log line stamps).

#include <string>
#include <string_view>

namespace user_common_213309941_213727837 {

// ISO-8601 UTC instant, e.g. "2026-05-30T23:31:10Z".
// Used for the `generated_at_utc` field required by the assignment's YAML reports.
[[nodiscard]] std::string utcTimestamp();

// Compact UTC stamp safe for file and directory names, e.g. "20260530_233110".
[[nodiscard]] std::string utcStampForFilename();

// Monotonically increasing suffix, e.g. "20260530_233110_000003".
// The assignment asks for "code that will generate a new number per time to avoid
// collision with existing files"; the trailing counter guarantees uniqueness even when
// two directories are created inside the same second.
[[nodiscard]] std::string uniqueStamp();

// Replaces every character that is not [A-Za-z0-9._-] with '_' so that a plugin
// file name can be embedded in an output file name.
[[nodiscard]] std::string sanitizeForFilename(std::string_view text);

} // namespace user_common_213309941_213727837
