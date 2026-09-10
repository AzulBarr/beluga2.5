#ifndef BELUGASLAM_CORE_OUTPUT_SELECTION_HPP
#define BELUGASLAM_CORE_OUTPUT_SELECTION_HPP
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace belugaslam {
struct OutputPoseHypothesis { std::size_t id; double mass, x, y; };
struct OutputSelection {
  std::size_t map_index = 0, risk_index = 0;
  double map_position_risk = 0, minimum_position_risk = 0;
};

// A decision over the retained frontend point poses; not a new PF update.
// Minimize sum_j w_j ||p_i-p_j||^2 over existing positive-mass hypotheses.
// Publishing an existing pose keeps the output paired with one complete map.
// The risk is an internal squared-position loss (m^2), NOT measured pose RMSE.
inline OutputSelection select_output_pose(const std::vector<OutputPoseHypothesis>& hypotheses) {
  if (hypotheses.empty()) throw std::invalid_argument("Empty output belief");
  long double total = 0;
  OutputSelection result;
  bool has_map = false;
  for (std::size_t i = 0; i < hypotheses.size(); ++i) {
    const auto& h = hypotheses[i];
    if (!std::isfinite(h.mass) || h.mass < 0 || !std::isfinite(h.x) || !std::isfinite(h.y))
      throw std::invalid_argument("Invalid output belief");
    total += h.mass;
    if (h.mass > 0 && (!has_map || h.mass > hypotheses[result.map_index].mass ||
        (h.mass == hypotheses[result.map_index].mass && h.id < hypotheses[result.map_index].id))) {
      result.map_index = i; has_map = true;
    }
  }
  if (!has_map || !std::isfinite(total)) throw std::invalid_argument("Empty output mass");
  const auto risk = [&](std::size_t i) {
    long double value = 0;
    for (const auto& h : hypotheses) {
      const long double dx = static_cast<long double>(hypotheses[i].x) - h.x;
      const long double dy = static_cast<long double>(hypotheses[i].y) - h.y;
      value += (static_cast<long double>(h.mass) / total) * (dx*dx + dy*dy);
    }
    if (!std::isfinite(value) || value > std::numeric_limits<double>::max())
      throw std::invalid_argument("Output risk overflow");
    return value;
  };
  // Start with MAP: exact ties retain its established pose / heading / map.
  // Sort candidate input by stable ID in the caller for deterministic non-MAP ties.
  result.risk_index = result.map_index;
  auto best = risk(result.map_index);
  result.map_position_risk = static_cast<double>(best);
  for (std::size_t i = 0; i < hypotheses.size(); ++i) {
    if (!(hypotheses[i].mass > 0)) continue;
    const auto value = risk(i);
    if (value < best) { result.risk_index = i; best = value; }
  }
  result.minimum_position_risk = static_cast<double>(best);
  return result;
}
}  // namespace belugaslam
#endif
