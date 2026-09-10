#include "belugaslam_core/loop_belief.hpp"

#include <cstdlib>
#include <iostream>

int main() {
  int failures = 0, checks = 0;
  const auto check = [&](bool condition, const char* name) {
    ++checks;
    if (!condition) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
  };
  const auto close = [](double a, double b) { return std::abs(a - b) < 1.0e-10; };
  using belugaslam::marginalize_compatibilities;
  const auto secondary = marginalize_compatibilities({0.6, 0.4}, {0.0, 1.0});
  check(close(secondary.weighted, 0.4), "secondary mode contributes its posterior mass");
  check(close(secondary.map, 0.0), "MAP-only verifier misses secondary support");
  check(secondary.weighted >= 0.25 && secondary.map < 0.25, "decision separation at fixed threshold");
  const auto unsupported = marginalize_compatibilities({0.6, 0.4}, {0.01, 0.02});
  check(unsupported.weighted < 0.25, "all incompatible trajectories reject a geometric candidate");
  const auto tiny = marginalize_compatibilities({0.99, 0.01}, {0.0, 1.0});
  check(close(tiny.weighted, 0.01), "tiny supporting mode is not renormalized to certainty");
  check(close(tiny.uniform, 0.5), "uniform averaging exposes the wrong conclusion for a tiny mode");
  const auto unavailable = marginalize_compatibilities({0.6, 0.4}, {std::numeric_limits<double>::quiet_NaN(), 1.0});
  check(close(unavailable.weighted, 0.4), "unavailable trajectory retains zero support and its denominator mass");
  check(close(marginalize_compatibilities({6.0, 4.0}, {0.0, 1.0}).weighted, 0.4), "raw masses are normalized");
  check(close(marginalize_compatibilities({1.0}, {0.73}).weighted, 0.73), "single-mode case reduces to its verifier");
  check(close(marginalize_compatibilities({0.4, 0.6}, {1.0, 0.0}).weighted, secondary.weighted), "weighted evidence is permutation invariant");
  check(close(marginalize_compatibilities({0.6, 0.2, 0.2}, {0.0, 1.0, 1.0}).weighted, secondary.weighted), "splitting an identical mode does not create extra evidence");

  const std::vector<belugaslam::PoseSample2> prior{{0, 0, 0}, {1, 0, 0.2}, {1, 1, 1.5}, {0, 1, 3.0}};
  std::vector<belugaslam::PoseSample2> transformed;
  const double angle = 0.7, c = std::cos(angle), s = std::sin(angle);
  for (const auto& pose : prior) transformed.push_back({c * pose.x - s * pose.y + 8, s * pose.x + c * pose.y - 9, pose.yaw + angle});
  const auto rigid = belugaslam::aligned_trajectory_change(prior, transformed);
  check(rigid.valid && rigid.translation_rmse < 1.0e-10, "global SE2 gauge change does not count as distortion");
  check(rigid.rotation_rmse < 1.0e-10, "headings align with the same gauge rotation");
  check(close(belugaslam::trajectory_compatibility(rigid, 0.3, 0.1), 1.0), "undistorted trajectory has unit compatibility");
  auto distorted = prior;
  distorted.back().x += 1.0;
  const auto deformation = belugaslam::aligned_trajectory_change(prior, distorted);
  check(deformation.valid && deformation.translation_rmse > 0.1, "nonrigid deformation survives alignment");
  auto stretched = prior;
  for (auto& pose : stretched) { pose.x *= 2.0; pose.y *= 2.0; }
  check(belugaslam::aligned_trajectory_change(prior, stretched).translation_rmse > 0.5, "metric scaling cannot be hidden by alignment");
  auto reversed_heading = prior;
  reversed_heading[1].yaw += 1.0;
  check(belugaslam::aligned_trajectory_change(prior, reversed_heading).rotation_rmse > 0.4, "heading deformation is measured");
  check(!belugaslam::aligned_trajectory_change({{0, 0, 0}}, {{0, 0, 0}}).valid, "insufficient trajectory is unavailable");
  check(!belugaslam::aligned_trajectory_change({{0,0,0},{0,0,0},{0,0,0}}, {{0,0,0},{0,0,0},{0,0,0}}).valid, "stationary trajectory cannot certify a loop");
  auto invalid = prior; invalid[0].x = std::numeric_limits<double>::infinity();
  check(!belugaslam::aligned_trajectory_change(prior, invalid).valid, "nonfinite trial is rejected");
  check(close(belugaslam::trajectory_compatibility({}, 0.3, 0.1), 0.0), "unavailable alignment gives zero compatibility");

  for (std::size_t budget : {4U, 5U, 30U, 31U}) {
    const std::vector<double> masses{0.90, 0.07, 0.02, 0.01};
    const auto quotas = belugaslam::allocate_particle_quotas(masses, budget);
    check(std::accumulate(quotas.begin(), quotas.end(), std::size_t{0}) == budget, "integer quotas exactly respect budget");
    check(*std::min_element(quotas.begin(), quotas.end()) >= 1, "every retained hypothesis has a particle");
    for (std::size_t h = 0; h < quotas.size(); ++h) {
      check(close((masses[h] / quotas[h]) * quotas[h], masses[h]), "minimum particle quota does not inflate posterior mass");
    }
  }
  bool rejected = false;
  try { (void)belugaslam::allocate_particle_quotas({0.5, 0.5}, 1); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "impossible budget is rejected");
  rejected = false;
  try { (void)marginalize_compatibilities({1.0}, {0.0, 1.0}); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "mismatched evidence arrays are rejected");
  if (failures) return EXIT_FAILURE;
  std::cout << "PASS: " << checks << " loop-belief, alignment and particle-mass checks\n";
  return EXIT_SUCCESS;
}
