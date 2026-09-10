#ifndef BELUGASLAM_CORE_PARTICLE_PROPOSAL_HPP
#define BELUGASLAM_CORE_PARTICLE_PROPOSAL_HPP
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>
namespace belugaslam {
struct ProposalSelection { std::size_t index=0; double log_evidence=0; };
// K independent draws from the motion prior. Select using their sensor likelihood;
// the ancestor's incremental importance weight is their MEAN likelihood, not the
// selected/maximized likelihood. K=1 is the bootstrap filter update.
template<class Generator>
ProposalSelection select_motion_proposal(const std::vector<double>& logs,Generator& generator) {
  if (logs.empty()) throw std::invalid_argument("Empty proposal set");
  const double maximum=*std::max_element(logs.begin(),logs.end());
  if (!std::isfinite(maximum)) throw std::invalid_argument("Non-finite proposal likelihood");
  std::vector<double> weights; weights.reserve(logs.size());
  double total=0;
  for (double l:logs) {
    if (!std::isfinite(l)) throw std::invalid_argument("Non-finite proposal likelihood");
    weights.push_back(std::exp(l-maximum));total+=weights.back();
  }
  std::discrete_distribution<std::size_t> choose(weights.begin(),weights.end());
  return {choose(generator),maximum+std::log(total/logs.size())};
}
// One random offset per ancestor population, with lower variance than independent
// categorical draws. Weights may be unnormalized; zero-weight bins are skipped.
template<class Generator>
std::vector<std::size_t> systematic_indices(const std::vector<double>& weights,std::size_t count,Generator& generator) {
  if (weights.empty() || count==0) return {};
  double total=0;
  for (double w:weights) {if (!std::isfinite(w)||w<0) throw std::invalid_argument("Invalid resampling weight"); total+=w;}
  if (!(total>0)||!std::isfinite(total)) throw std::invalid_argument("Empty resampling mass");
  std::uniform_real_distribution<double> uniform(0,1);
  const double offset=uniform(generator);
  std::size_t index=0;double cumulative=weights[0]/total;
  std::vector<std::size_t> result;result.reserve(count);
  for (std::size_t n=0;n<count;++n) {
    const double position=(n+offset)/count;
    while (index+1<weights.size() && position>=cumulative) cumulative+=weights[++index]/total;
    result.push_back(index);
  }
  return result;
}
}  // namespace belugaslam
#endif
