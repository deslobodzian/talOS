// contract.bzl's talos_topic_problem (run by talos_messages at load time)
// against naming::CheckTopicName, the authority: a name one accepts and the
// other refuses would build a topics.h the launcher then rejects, or reject a
// name the launcher allows. Besides the listed names, the Starlark tables of
// roles, abbreviations and direction suffixes must hold exactly names.h's.

#include <cstdlib>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/introspection/names.h"

namespace naming = talos::introspect::naming;

namespace {

struct Probe {
  std::string kind;
  std::string entry;
  bool starlark_ok{false};
  std::string topic;
};

std::vector<Probe> ReadProbes() {
  const char* path = std::getenv("TALOS_TOPIC_VERDICTS");
  EXPECT_NE(path, nullptr);
  std::vector<Probe> probes;
  if (path == nullptr) return probes;
  std::ifstream verdicts{path};
  EXPECT_TRUE(verdicts.is_open()) << path;
  std::string line;
  while (std::getline(verdicts, line)) {
    // `<kind> <entry> <ok|bad> <topic>`; the topic may be empty.
    const std::size_t a = line.find(' ');
    const std::size_t b = line.find(' ', a + 1);
    const std::size_t c = line.find(' ', b + 1);
    if (a == std::string::npos || b == std::string::npos ||
        c == std::string::npos) {
      ADD_FAILURE() << "malformed line '" << line << "'";
      continue;
    }
    probes.push_back({.kind = line.substr(0, a),
                      .entry = line.substr(a + 1, b - a - 1),
                      .starlark_ok = line.substr(b + 1, c - b - 1) == "ok",
                      .topic = line.substr(c + 1)});
  }
  return probes;
}

std::set<std::string> Entries(const std::vector<Probe>& probes,
                              const std::string& kind, bool expect_ok) {
  std::set<std::string> out;
  for (const Probe& probe : probes) {
    if (probe.kind != kind) continue;
    out.insert(probe.entry);
    EXPECT_EQ(probe.starlark_ok, expect_ok)
        << kind << " " << probe.entry << ": " << probe.topic;
  }
  return out;
}

TEST(TopicGrammar, StarlarkAgreesWithCheckTopicName) {
  const std::vector<Probe> probes = ReadProbes();
  int accepted = 0;
  int rejected = 0;
  for (const Probe& probe : probes) {
    const auto problem = naming::CheckTopicName(probe.topic);
    EXPECT_EQ(probe.starlark_ok, !problem.has_value())
        << "'" << probe.topic << "': " << problem.value_or("accepted by C++");
    if (probe.kind == "name") (probe.starlark_ok ? accepted : rejected) += 1;
  }
  // Both verdicts occur among the listed names, so an always-accept or
  // always-reject Starlark check cannot pass.
  EXPECT_GE(accepted, 5);
  EXPECT_GE(rejected, 10);
}

TEST(TopicGrammar, StarlarkTablesMatchNamesH) {
  const std::vector<Probe> probes = ReadProbes();

  std::set<std::string> roles;
  for (const auto role : naming::kOwnerPublishedRoles) roles.emplace(role);
  for (const auto role : naming::kOwnerConsumedRoles) roles.emplace(role);
  EXPECT_EQ(Entries(probes, "role", /*expect_ok=*/true), roles);

  std::set<std::string> abbreviations;
  for (const auto& [wrong, right] : naming::kAbbreviations) {
    abbreviations.insert(std::string{wrong} + ">" + std::string{right});
  }
  EXPECT_EQ(Entries(probes, "abbreviation", /*expect_ok=*/false),
            abbreviations);

  std::set<std::string> suffixes;
  for (const auto suffix : naming::kDirectionSuffixes) suffixes.emplace(suffix);
  EXPECT_EQ(Entries(probes, "suffix", /*expect_ok=*/false), suffixes);
}

}  // namespace
