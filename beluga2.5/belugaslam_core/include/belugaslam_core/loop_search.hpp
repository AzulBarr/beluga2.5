#ifndef BELUGASLAM_CORE_LOOP_SEARCH_HPP
#define BELUGASLAM_CORE_LOOP_SEARCH_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <vector>
#include "loop_belief.hpp"

namespace belugaslam {

// Read the immutable, already cached field. Cell-center interpolation removes
// the half-cell plateaus of floor-index scoring without allocating another grid.
class LoopFieldView {
 public:
  LoopFieldView(const std::vector<float>& distances, const std::vector<double>& scores,
                int width, int height, double resolution, double ox, double oy)
      : distances_(distances), scores_(scores), width_(width), height_(height),
        resolution_(resolution), ox_(ox), oy_(oy) {
    if (width < 2 || height < 2 || !std::isfinite(resolution) || resolution <= 0 ||
        !std::isfinite(ox) || !std::isfinite(oy) ||
        distances.size() != static_cast<std::size_t>(width)*height || scores.size() != distances.size())
      throw std::invalid_argument("Invalid loop field");
  }
  struct Sample { double distance = INFINITY, score = 0; };
  [[nodiscard]] Sample sample(double x, double y) const {
    const double fx=(x-ox_)/resolution_-0.5, fy=(y-oy_)/resolution_-0.5;
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx<0 || fy<0 || fx>=width_-1 || fy>=height_-1) return {};
    const int ix=static_cast<int>(fx), iy=static_cast<int>(fy);
    const double ax=fx-ix, ay=fy-iy;
    const auto index=static_cast<std::size_t>(iy)*width_+ix;
    const auto interpolate=[&](const auto& v) {
      return (1-ay)*((1-ax)*v[index]+ax*v[index+1])+
                 ay*((1-ax)*v[index+width_]+ax*v[index+width_+1]);
    };
    return {interpolate(distances_),interpolate(scores_)};
  }
 private:
  const std::vector<float>& distances_;
  const std::vector<double>& scores_;
  int width_,height_;
  double resolution_,ox_,oy_;
};

struct LoopScore { double score=0, overlap=0; };
template<class Points>
inline LoopScore score_loop_scan(const LoopFieldView& field, const Points& scan, const PoseSample2& pose) {
  LoopScore result;
  if (scan.empty()) return result;
  const double c=std::cos(pose.yaw), s=std::sin(pose.yaw);
  for (const auto& point:scan) {
    const auto value=field.sample(pose.x+c*point.first-s*point.second,pose.y+s*point.first+c*point.second);
    result.score+=value.score;
    if (value.distance<=0.30) result.overlap+=1;
  }
  result.score/=scan.size(); result.overlap/=scan.size();
  return result;
}

struct LoopSearchOptions {
  double translation_window=3.0, rotation_window=0.7;
  double min_score=0.55, min_overlap=0.35;
  std::size_t beam_width=8, max_modes=2;
};
struct LoopSearchMatch { PoseSample2 pose{}; LoopScore value; };

inline bool separated_loop_modes(const PoseSample2& a,const PoseSample2& b,double distance,double angle) {
  return std::hypot(a.x-b.x,a.y-b.y)>=distance || std::abs(wrap_angle(a.yaw-b.yaw))>=angle;
}

// Fixed work per search window: <=41*41*47 coarse poses, eight independent
// refinement paths, seven halving levels. Each coarse basin keeps its own path;
// coincident fine poses cannot consume another basin's refinement slots.
// This is a bounded heuristic search, not a branch-and-bound optimality claim.
template<class Score>
inline std::vector<LoopSearchMatch> search_loop_modes(
    const PoseSample2& initial,const LoopSearchOptions& o,Score score) {
  if (!std::isfinite(o.translation_window) || o.translation_window<0 || o.translation_window>10 ||
      !std::isfinite(o.rotation_window) || o.rotation_window<0 || o.rotation_window>3.141592653589793 ||
      !(o.min_score>=0 && o.min_score<=1) || !(o.min_overlap>=0 && o.min_overlap<=1) ||
      o.beam_width<1 || o.beam_width>8 || o.max_modes<1 || o.max_modes>o.beam_width)
    throw std::invalid_argument("Invalid bounded loop search options");
  if (!std::isfinite(initial.x) || !std::isfinite(initial.y) || !std::isfinite(initial.yaw)) return {};
  struct Seed { double x,y,a; LoopSearchMatch match; };
  const int nt=static_cast<int>(std::ceil(o.translation_window/0.5));
  const int na=static_cast<int>(std::ceil(o.rotation_window/0.14));
  const double dt=nt?o.translation_window/nt:0, da=na?o.rotation_window/na:0;
  const double c=std::cos(initial.yaw), s=std::sin(initial.yaw);
  auto evaluate=[&](double x,double y,double a) {
    PoseSample2 pose{initial.x+c*x-s*y,initial.y+s*x+c*y,wrap_angle(initial.yaw+a)};
    return Seed{x,y,a,{pose,score(pose)}};
  };
  auto rank=[](const Seed& seed) {return seed.match.value.score*std::min(1.0,seed.match.value.overlap/0.5);};
  // A flat geometric direction must not choose a search-window corner simply
  // because it was enumerated first. This is a tie break, not an evidence factor.
  auto better=[&](const Seed& a,const Seed& b) {
    const double ar=rank(a),br=rank(b);
    if (ar!=br) return ar>br;
    return a.x*a.x+a.y*a.y+a.a*a.a < b.x*b.x+b.y*b.y+b.a*b.a;
  };
  auto finite=[](const Seed& seed) {return std::isfinite(seed.match.value.score) && std::isfinite(seed.match.value.overlap);};
  std::vector<Seed> coarse;
  for (int ix=-nt;ix<=nt;++ix) for (int iy=-nt;iy<=nt;++iy) for (int ia=-na;ia<=na;++ia) {
    auto seed=evaluate(ix*dt,iy*dt,ia*da);
    if (finite(seed) && seed.match.value.overlap>=o.min_overlap*0.75) coarse.push_back(seed);
  }
  std::stable_sort(coarse.begin(),coarse.end(),better);
  std::vector<Seed> seeds;
  for (const auto& seed:coarse) {
    const bool distinct=std::all_of(seeds.begin(),seeds.end(),[&](const auto& kept) {
      return separated_loop_modes(seed.match.pose,kept.match.pose,0.50,0.14);
    });
    if (distinct) seeds.push_back(seed);
    if (seeds.size()==o.beam_width) break;
  }
  std::vector<Seed> refined;
  for (auto seed:seeds) {
    double step_t=dt/2, step_a=da/2;
    for (int level=0;level<7;++level,step_t*=0.5,step_a*=0.5) {
      auto best=seed;
      for (int ix=nt?-1:0;ix<=(nt?1:0);++ix) for (int iy=nt?-1:0;iy<=(nt?1:0);++iy)
        for (int ia=na?-1:0;ia<=(na?1:0);++ia) {
          if (ix==0 && iy==0 && ia==0) continue;
          const double x=seed.x+ix*step_t, y=seed.y+iy*step_t, a=seed.a+ia*step_a;
          if (std::abs(x)>o.translation_window || std::abs(y)>o.translation_window || std::abs(a)>o.rotation_window) continue;
          const auto candidate=evaluate(x,y,a);
          if (finite(candidate) && better(candidate,best)) best=candidate;
        }
      seed=best;
    }
    if (seed.match.value.score>=o.min_score && seed.match.value.overlap>=o.min_overlap) refined.push_back(seed);
  }
  std::stable_sort(refined.begin(),refined.end(),better);
  std::vector<LoopSearchMatch> result;
  for (const auto& seed:refined) {
    if (std::all_of(result.begin(),result.end(),[&](const auto& kept) {
        return separated_loop_modes(seed.match.pose,kept.pose,0.35,0.12);
    })) result.push_back(seed.match);
    if (result.size()==o.max_modes) break;
  }
  return result;
}

// A query's loop/no-loop decision consumes that scan's evidence once, including
// in descendants that chose no loop. Ownership belongs to the live belief, not
// to an individual branch. Retains one integer per consumed event, like the graph.
class LoopQueryLedger {
 public:
  [[nodiscard]] bool contains(std::uint64_t query) const {return queries_.count(query)!=0;}
  bool consume(std::uint64_t query) {return queries_.insert(query).second;}
  [[nodiscard]] std::size_t size() const {return queries_.size();}
 private:
  std::set<std::uint64_t> queries_;
};
}  // namespace belugaslam
#endif
