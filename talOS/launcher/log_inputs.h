#pragma once

// The log files a log tool's inputs name (log_dump, tlog_to_wpilog).

#include <string>
#include <vector>

namespace talos::launcher {

// The .tlog files `inputs` name, in input order: a directory is every *.tlog
// in it (FindLogFiles), a log file is that log's kept segments too, oldest
// first (LogSegmentFiles), and a path that is neither is kept as given, to
// fail, named, when opened. Each file appears once, by canonical path,
// however many inputs name it (a session directory and a file in it, or one
// file twice), so no log is merged twice.
std::vector<std::string> LogInputFiles(const std::vector<std::string>& inputs);

}  // namespace talos::launcher
