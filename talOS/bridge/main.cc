// CLI entry point lives in node.cc so component tests in trust_test can
// exercise flag parsing and exit status directly without spawning a subprocess.
namespace talos::hardware {
int RunHardwareNodeMain(int argc, char** argv);
}  // namespace talos::hardware

int main(int argc, char** argv) {
  return talos::hardware::RunHardwareNodeMain(argc, argv);
}
