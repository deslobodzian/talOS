#pragma once

// Shared by node_api.cc and node_api_testing.cc: error capture and the
// callback adapters.

#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "talOS/node/node.h"
#include "talOS/node_api/node_api.h"

namespace talos::node_api {

void SetError(std::string message);
void ClearError();

// Runs `body`, turning any exception into `failure` plus talos_last_error().
template <class R, class F>
R Guard(R failure, F&& body) noexcept {
  try {
    ClearError();
    return std::forward<F>(body)();
  } catch (const std::exception& e) {
    SetError(e.what());
  } catch (...) {
    SetError("unknown error");
  }
  return failure;
}

inline Node& AsNode(TalosNode* node) {
  if (node == nullptr) throw std::invalid_argument("null TalosNode");
  return *reinterpret_cast<Node*>(node);
}
inline const Node& AsNode(const TalosNode* node) {
  if (node == nullptr) throw std::invalid_argument("null TalosNode");
  return *reinterpret_cast<const Node*>(node);
}
inline TalosNode* AsHandle(Node& node) {
  return reinterpret_cast<TalosNode*>(&node);
}

inline const char* NonNull(const char* text, const char* what) {
  if (text == nullptr) {
    throw std::invalid_argument(std::string{what} + " is NULL");
  }
  return text;
}

// A callback's non-zero return becomes an exception on the C++ side; the
// caller keeps the real error.
inline void Check(std::int32_t status, const char* what) {
  if (status != 0) throw std::runtime_error(std::string{what} + " failed");
}

// The caller builds and owns its object; the framework holds nothing.
inline detail::InstanceFactory MakeFactory(TalosFactory factory, void* user) {
  if (factory == nullptr) throw std::invalid_argument("null TalosFactory");
  return [factory, user](Node& node) -> detail::Instance {
    Check(factory(AsHandle(node), user), "node factory");
    return nullptr;
  };
}

}  // namespace talos::node_api
