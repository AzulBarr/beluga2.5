#include <cmath>
#include <cstdlib>
#include <iostream>

#include "belugaslam_core/motion_filter.hpp"

// Dependency-free executable: also runnable with only a C++17 compiler.
int main() {
  int failures = 0;
  const auto check = [&failures](bool condition, const char* name) {
    if (!condition) {
      std::cerr << "FAIL: " << name << '\n';
      ++failures;
    }
  };
  const auto accepts = [](double dt, double distance, double angle) {
    return motion_filter_accepts(dt, distance, angle, 5.0, 0.15, 0.08);
  };
  check(!accepts(0.0, 0.0, 0.0), "identical scan is filtered");
  check(!accepts(5.0, 0.15, 0.08), "equality is filtered on all three thresholds");
  check(accepts(std::nextafter(5.0, 6.0), 0.0, 0.0), "time alone triggers insertion");
  check(accepts(0.0, std::nextafter(0.15, 1.0), 0.0), "translation alone triggers insertion");
  check(accepts(0.0, 0.0, std::nextafter(0.08, 1.0)), "rotation alone triggers insertion");
  check(accepts(-1.0, 0.0, 0.0), "clock rewind restarts insertion interval");
  check(!motion_filter_accepts(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), "duplicate at zero limits is filtered");
  check(motion_filter_accepts(0.01, 0.0, 0.0, 0.0, 0.0, 0.0), "zero time limit accepts next timestamp");

  // Many individually small movements must accumulate from the last INSERTION,
  // and rejected timestamps must not postpone the maximum-time insertion.
  double inserted_x = 0.0;
  double inserted_time = 0.0;
  int insertions = 1;  // first scan is accepted by the caller
  for (int i = 1; i <= 3; ++i) {
    const double x = 0.06 * i;
    if (accepts(i - inserted_time, x - inserted_x, 0.0)) {
      inserted_x = x;
      inserted_time = i;
      ++insertions;
    }
  }
  check(insertions == 2, "small movements accumulate to one new insertion");
  check(inserted_time == 3.0, "last inserted timestamp advances only on acceptance");
  check(!accepts(8.0 - inserted_time, 0.0, 0.0), "stationary exact timeout is filtered");
  check(accepts(8.1 - inserted_time, 0.0, 0.0), "stationary timeout is not reset by filtered scans");
  if (failures != 0) return EXIT_FAILURE;
  std::cout << "PASS: 12 motion-filter checks\n";
  return EXIT_SUCCESS;
}
