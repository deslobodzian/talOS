#pragma once

#include <ostream>
#include <sstream>
#include <string>

namespace talos::launcher {

// One launcher console line, handed to its stream in one write.
//
// While a session runs, std::cout is LauncherOutput's unbuffered sink, which
// also takes members' output echoed from another thread; every `<<` is one
// append there, so a `std::cout << a << b` chain lets a member's line land
// between `a` and `b`. ConsoleLine composes the whole text and writes it at
// the end of the full expression (or of its scope, when named), so the line
// enters the stream whole:
//
//   ConsoleLine(std::cout) << "[launcher] Started " << name << "\n";
class ConsoleLine {
 public:
  explicit ConsoleLine(std::ostream& to) : to_{to} {}
  ~ConsoleLine() {
    const std::string text = text_.str();
    to_.write(text.data(), static_cast<std::streamsize>(text.size()));
    to_.flush();
  }
  ConsoleLine(const ConsoleLine&) = delete;
  ConsoleLine& operator=(const ConsoleLine&) = delete;
  ConsoleLine(ConsoleLine&&) = delete;
  ConsoleLine& operator=(ConsoleLine&&) = delete;

  template <typename T>
  ConsoleLine& operator<<(const T& value) {
    text_ << value;
    return *this;
  }
  ConsoleLine& operator<<(std::ostream& (*manipulator)(std::ostream&)) {
    text_ << manipulator;
    return *this;
  }

 private:
  std::ostream& to_;
  std::ostringstream text_;
};

}  // namespace talos::launcher
