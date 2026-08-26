// TinyNPY compatibility shim.
// The skeleton's Map3DImpl.h references TinyNPY::Array.
// We expose our NpyArray under that name so the skeleton-mandated
// header compiles without pulling in the full TinyNPY external library.
#pragma once

#include <drone_mapper/NpyArray.h>

namespace TinyNPY {
    using Array = drone_mapper::NpyArray;
} // namespace TinyNPY
