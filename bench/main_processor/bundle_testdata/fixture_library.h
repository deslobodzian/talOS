// bundle_test only: a function in a shared library (libfixture_library.so)
// that the linked_library node calls, so its binary links the library
// dynamically and its bundle stages the library in lib/.
#ifndef BENCH_MAIN_PROCESSOR_BUNDLE_TESTDATA_FIXTURE_LIBRARY_H_
#define BENCH_MAIN_PROCESSOR_BUNDLE_TESTDATA_FIXTURE_LIBRARY_H_

namespace bundle_testdata {

int FixtureLibraryAnswer();

}  // namespace bundle_testdata

#endif  // BENCH_MAIN_PROCESSOR_BUNDLE_TESTDATA_FIXTURE_LIBRARY_H_
