#pragma once

#include <flatbuffers/flatbuffers.h>

#include <type_traits>

namespace ipc {
template <typename T>
concept NotDerivedFromFlatbufferTable =
    !std::is_base_of_v<flatbuffers::Table, T>;

}  // namespace ipc
