#pragma once

#include <type_traits>
#include <flatbuffers/flatbuffers.h>

namespace ipc {
template <typename T>
concept NotDerivedFromFlatbufferTable = !std::is_base_of_v<flatbuffers::Table, T>;

} /* namespace icp */
