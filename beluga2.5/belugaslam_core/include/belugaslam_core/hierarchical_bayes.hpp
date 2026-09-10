#ifndef BELUGASLAM_CORE_HIERARCHICAL_BAYES_HPP
#define BELUGASLAM_CORE_HIERARCHICAL_BAYES_HPP
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include <utility>

namespace belugaslam {
inline double log_sum_exp(const std::vector<double>& values) {
  if (values.empty()) return -std::numeric_limits<double>::infinity();
  for (double v : values)
    if (std::isnan(v) || v == std::numeric_limits<double>::infinity())
      throw std::invalid_argument("Invalid log weight");
  const double maximum = *std::max_element(values.begin(), values.end());
  if (!std::isfinite(maximum)) return maximum;
  double sum = 0;
  for (double v : values) sum += std::exp(v - maximum);
  return maximum + std::log(sum);
}

// Returns the discarded normalizer. All-zero support is an error, not a
// request to resurrect every impossible state with a uniform distribution.
inline double normalize_log_weights(std::vector<double>& logs) {
  const double normalizer = log_sum_exp(logs);
  if (!std::isfinite(normalizer)) throw std::invalid_argument("Empty posterior support");
  for (auto& l : logs) l -= normalizer;
  return normalizer;
}

struct ConditionalUpdate {
  double log_evidence = 0;
  std::vector<double> log_weights;
};

// Each incremental likelihood can itself be a Monte Carlo MEAN over motion
// proposals. Normalize the prior first: duplicating particles or reallocating
// their integer budget cannot multiply the predictive evidence.
inline ConditionalUpdate update_conditional(
    std::vector<double> log_prior, const std::vector<double>& log_likelihood) {
  if (log_prior.size() != log_likelihood.size() || log_prior.empty())
    throw std::invalid_argument("Mismatched conditional population");
  normalize_log_weights(log_prior);
  for (std::size_t i = 0; i < log_prior.size(); ++i) log_prior[i] += log_likelihood[i];
  ConditionalUpdate result;
  result.log_evidence = normalize_log_weights(log_prior);
  result.log_weights = std::move(log_prior);
  return result;
}

enum class LoopDecision { pending, accepted, rejected, undecided };
inline const char* decision_name(LoopDecision d) {
  switch (d) {
    case LoopDecision::accepted: return "accepted";
    case LoopDecision::rejected: return "rejected";
    case LoopDecision::undecided: return "undecided";
    default: return "pending";
  }
}
struct SequentialLoopOptions {
  std::size_t min_scans = 10, max_scans = 30;
  double accept_probability = .95, reject_probability = .05;
  double beta = .1; // multiplies tracking.effective_beams, not a second factor
  double min_known_fraction = .35;
  void validate() const {
    if (min_scans == 0 || max_scans < min_scans ||
        !(reject_probability > 0 && reject_probability < .5) ||
        !(accept_probability > .5 && accept_probability < 1) ||
        !(beta > 0 && beta <= 1) ||
        !(min_known_fraction > 0 && min_known_fraction <= 1))
      throw std::invalid_argument("Invalid sequential loop evidence parameters");
  }
};
inline LoopDecision decide_loop(double probability, std::size_t evidence_scans,
                                std::size_t attempted_scans, const SequentialLoopOptions& o) {
  if (!std::isfinite(probability) || probability < 0 || probability > 1)
    throw std::invalid_argument("Invalid loop posterior");
  if (evidence_scans >= o.min_scans) {
    if (probability >= o.accept_probability) return LoopDecision::accepted;
    if (probability <= o.reject_probability) return LoopDecision::rejected;
  }
  return attempted_scans >= o.max_scans ? LoopDecision::undecided : LoopDecision::pending;
}
} // namespace belugaslam
#endif
