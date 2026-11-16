#include "talOS/launcher/console_line.h"

#include <gtest/gtest.h>

#include <ostream>
#include <streambuf>
#include <string>
#include <vector>

namespace talos::launcher {
namespace {

// Records each call that hands it bytes, as LauncherOutput's sink takes each
// one as a separate append another thread's echo may come between.
class Writes : public std::streambuf {
 public:
  std::vector<std::string> calls;

 protected:
  int_type overflow(int_type c) override {
    if (!traits_type::eq_int_type(c, traits_type::eof()))
      calls.emplace_back(1, traits_type::to_char_type(c));
    return traits_type::not_eof(c);
  }
  std::streamsize xsputn(const char* data, std::streamsize size) override {
    calls.emplace_back(data, static_cast<std::size_t>(size));
    return size;
  }
};

// REQ-OPS-33
// A launcher line made of several pieces reaches its stream in one write,
// so a member's echoed line cannot land inside it (seen in CI: "[launcher]
// Started sim_gateway (pid 20990, target //talOS/simulation:sim_gateway"
// with "hardware_node: gateway boot ..." spliced in before its ")").
TEST(ConsoleLine, AComposedLineIsOneWrite) {
  Writes sink;
  std::ostream out{&sink};
  const int pid = 20990;
  ConsoleLine(out) << "[launcher] Started " << "sim_gateway" << " (pid " << pid
                   << ", target //talOS/simulation:sim_gateway" << ")\n";
  {
    ConsoleLine line{out};
    line << "[launcher] cannot spawn x (e); ";
    line << "retry in " << 5 << " ms" << '\n' << std::flush;
  }
  ASSERT_EQ(sink.calls.size(), 2U);
  EXPECT_EQ(sink.calls[0],
            "[launcher] Started sim_gateway (pid 20990, target "
            "//talOS/simulation:sim_gateway)\n");
  EXPECT_EQ(sink.calls[1], "[launcher] cannot spawn x (e); retry in 5 ms\n");
}

}  // namespace
}  // namespace talos::launcher
