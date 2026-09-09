#ifndef BELUGASLAM_CORE_ROBUST_TRACKING_HPP
#define BELUGASLAM_CORE_ROBUST_TRACKING_HPP
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "loop_belief.hpp"

namespace belugaslam {
using ScanPoints = std::vector<std::pair<double, double>>;
struct TrackingOptions {
  double sigma = 0.15, outlier_probability = 0.05;
  double prior_translation_sigma = 0.50, prior_rotation_sigma = 0.20;
  double max_translation = 0.50, max_rotation = 0.25;
  double min_overlap = 0.35, inlier_distance = 0.20;
  double effective_beams = 20.0;
  // The matcher is a regularized point estimate, not the PF importance update.
  // Keep the established prior by default. 1/effective_beams is the explicitly
  // selectable Gaussian-prior / effective-beam likelihood interpretation.
  double prior_information_scale = 1.0;
  std::size_t min_points = 12, max_points = 180;
  int max_iterations = 20;
};
struct FieldSample { double distance = 1.0, dx = 0.0, dy = 0.0; };

// A bounded chamfer distance field in cell-center coordinates, shared read-only
// during one matching phase. Bilinear interpolation supplies analytic derivatives.
class TrackingField {
 public:
  TrackingField(const std::vector<float>& cells, int width, int height,
                double resolution, double origin_x, double origin_y)
      : width_(width), height_(height), resolution_(resolution), ox_(origin_x), oy_(origin_y) {
    if (width < 2 || height < 2 || !(resolution > 0) || !std::isfinite(resolution) ||
        !std::isfinite(ox_) || !std::isfinite(oy_) || cells.size() != static_cast<std::size_t>(width) * height)
      throw std::invalid_argument("Invalid tracking field extent");
    constexpr float infinity = 1e6F, diagonal = 1.41421356237F;
    distances_.assign(cells.size(), infinity);
    for (std::size_t i = 0; i < cells.size(); ++i) if (cells[i] > 0.65F) {
      distances_[i] = 0; ++occupied_;
    }
    auto at = [&](int x, int y) -> float& { return distances_[static_cast<std::size_t>(y * width + x)]; };
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
      auto& v = at(x,y);
      if (x) v = std::min(v, at(x-1,y)+1);
      if (y) v = std::min(v, at(x,y-1)+1);
      if (x && y) v = std::min(v, at(x-1,y-1)+diagonal);
      if (x+1 < width && y) v = std::min(v, at(x+1,y-1)+diagonal);
    }
    for (int y = height-1; y >= 0; --y) for (int x = width-1; x >= 0; --x) {
      auto& v = at(x,y);
      if (x+1 < width) v = std::min(v, at(x+1,y)+1);
      if (y+1 < height) v = std::min(v, at(x,y+1)+1);
      if (x+1 < width && y+1 < height) v = std::min(v, at(x+1,y+1)+diagonal);
      if (x && y+1 < height) v = std::min(v, at(x-1,y+1)+diagonal);
    }
    for (auto& v : distances_) v = static_cast<float>(std::min(1.0, v * resolution));
  }
  [[nodiscard]] std::size_t occupied_cells() const { return occupied_; }
  [[nodiscard]] FieldSample sample(double x, double y) const {
    const double fx = (x-ox_) / resolution_ - 0.5, fy = (y-oy_) / resolution_ - 0.5;
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0 || fy < 0 || fx >= width_-1 || fy >= height_-1) return {};
    const int ix = static_cast<int>(std::floor(fx)), iy = static_cast<int>(std::floor(fy));
    const double ax = fx-ix, ay = fy-iy;
    const auto offset = static_cast<std::size_t>(iy*width_+ix);
    const double a = distances_[offset], b = distances_[offset+1];
    const double c = distances_[offset+width_], d = distances_[offset+width_+1];
    return {(1-ay)*((1-ax)*a+ax*b)+ay*((1-ax)*c+ax*d),
            ((1-ay)*(b-a)+ay*(d-c))/resolution_, ((1-ax)*(c-a)+ax*(d-b))/resolution_};
  }
 private:
  int width_, height_;
  double resolution_, ox_, oy_;
  std::size_t occupied_ = 0;
  std::vector<float> distances_;
};
struct TrackingScore { double mean_log_likelihood = 0, overlap = 0; std::size_t inliers = 0; };
inline TrackingScore tracking_score(const TrackingField& field, const ScanPoints& scan,
                                    const PoseSample2& pose, const TrackingOptions& o) {
  TrackingScore score;
  if (scan.empty()) return score;
  const double c=std::cos(pose.yaw), s=std::sin(pose.yaw);
  for (const auto& [x,y] : scan) {
    const auto v=field.sample(pose.x+c*x-s*y,pose.y+s*x+c*y);
    const double gaussian=std::exp(-0.5*v.distance*v.distance/(o.sigma*o.sigma));
    score.mean_log_likelihood+=std::log(o.outlier_probability+(1-o.outlier_probability)*gaussian);
    if (v.distance<=o.inlier_distance) ++score.inliers;
  }
  score.mean_log_likelihood/=scan.size(); score.overlap=static_cast<double>(score.inliers)/scan.size();
  return score;
}
inline double tracking_objective(const TrackingField& f,const ScanPoints& z,const PoseSample2& p,
                                 const PoseSample2& prior,const TrackingOptions& o) {
  const double dx=(p.x-prior.x)/o.prior_translation_sigma, dy=(p.y-prior.y)/o.prior_translation_sigma;
  const double da=wrap_angle(p.yaw-prior.yaw)/o.prior_rotation_sigma;
  // Cost units are per beam. A scale of 1/B gives the Gaussian-prior MAP
  // interpretation for L=exp(B*mean_log_L); scale 1 preserves legacy smoothing.
  return -tracking_score(f,z,p,o).mean_log_likelihood+0.5*(dx*dx+dy*dy+da*da)*o.prior_information_scale;
}
inline bool solve_tracking_system(std::array<std::array<double,3>,3> matrix,
                                  std::array<double,3> rhs,std::array<double,3>& solution) {
  for (int col=0;col<3;++col) {
    int pivot=col;
    for (int row=col+1;row<3;++row) if (std::abs(matrix[row][col])>std::abs(matrix[pivot][col])) pivot=row;
    if (!std::isfinite(matrix[pivot][col]) || std::abs(matrix[pivot][col])<1e-14) return false;
    std::swap(matrix[pivot],matrix[col]); std::swap(rhs[pivot],rhs[col]);
    for (int row=col+1;row<3;++row) {
      const double factor=matrix[row][col]/matrix[col][col];
      for (int j=col;j<3;++j) matrix[row][j]-=factor*matrix[col][j];
      rhs[row]-=factor*rhs[col];
    }
  }
  for (int row=2;row>=0;--row) {
    double value=rhs[row];
    for (int j=row+1;j<3;++j) value-=matrix[row][j]*solution[j];
    solution[row]=value/matrix[row][row];
    if (!std::isfinite(solution[row])) return false;
  }
  return true;
}
struct TrackingResult {
  PoseSample2 pose{};
  TrackingScore score;
  bool accepted=false;
  double initial_cost=0, final_cost=0;
};
inline TrackingResult match_tracking_scan(const TrackingField& field,const ScanPoints& scan,
                                          const PoseSample2& prior,const TrackingOptions& o,
                                          const PoseSample2* seed = nullptr) {
  TrackingResult result; result.pose=prior;
  if (scan.empty() || field.occupied_cells()==0) return result;
  double cost=tracking_objective(field,scan,prior,prior,o);
  result.initial_cost=cost;
  auto pose=prior;
  if (seed && std::hypot(seed->x-prior.x,seed->y-prior.y)<=o.max_translation &&
      std::abs(wrap_angle(seed->yaw-prior.yaw))<=o.max_rotation) {
    const double seed_cost=tracking_objective(field,scan,*seed,prior,o);
    if (seed_cost<cost) {pose=*seed;cost=seed_cost;}
  }
  const auto around=pose;
  // A small fixed seed lattice improves the attraction basin; every seed is
  // evaluated with the same odometry prior, never recentered after a search level.
  for (double dx : {-0.12,0.0,0.12}) for (double dy : {-0.12,0.0,0.12}) for (double da : {-0.06,0.0,0.06}) {
    if (std::hypot(dx,dy)>o.max_translation || std::abs(da)>o.max_rotation) continue;
    PoseSample2 p{around.x+dx,around.y+dy,wrap_angle(around.yaw+da)};
    if (std::hypot(p.x-prior.x,p.y-prior.y)>o.max_translation ||
        std::abs(wrap_angle(p.yaw-prior.yaw))>o.max_rotation) continue;
    const auto candidate_cost=tracking_objective(field,scan,p,prior,o);
    if (candidate_cost<cost) {cost=candidate_cost;pose=p;}
  }
  double damping=1e-3;
  for (int iteration=0;iteration<o.max_iterations;++iteration) {
    std::array<std::array<double,3>,3> H{};
    std::array<double,3> g{},step{};
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    for (const auto& [x,y] : scan) {
      const double rx=c*x-s*y,ry=s*x+c*y;
      const auto v=field.sample(pose.x+rx,pose.y+ry);
      const double gaussian=std::exp(-0.5*v.distance*v.distance/(o.sigma*o.sigma));
      const double responsibility=(1-o.outlier_probability)*gaussian/(o.outlier_probability+(1-o.outlier_probability)*gaussian);
      const double weight=responsibility/(scan.size()*o.sigma*o.sigma);
      const std::array<double,3> J{v.dx,v.dy,-v.dx*ry+v.dy*rx};
      for (int a=0;a<3;++a) {
        g[a]+=weight*v.distance*J[a];
        for (int b=0;b<3;++b) H[a][b]+=weight*J[a]*J[b];
      }
    }
    const std::array<double,3> delta{pose.x-prior.x,pose.y-prior.y,wrap_angle(pose.yaw-prior.yaw)};
    for (int a=0;a<3;++a) {
      const double sigma=a==2?o.prior_rotation_sigma:o.prior_translation_sigma;
      const double information=o.prior_information_scale/(sigma*sigma);
      H[a][a]+=information;g[a]+=delta[a]*information;
      H[a][a]+=damping*std::max(1.0,H[a][a]);g[a]=-g[a];
    }
    if (!solve_tracking_system(H,g,step)) break;
    PoseSample2 candidate{pose.x+step[0],pose.y+step[1],wrap_angle(pose.yaw+step[2])};
    const double distance=std::hypot(candidate.x-prior.x,candidate.y-prior.y);
    if (distance>o.max_translation || std::abs(wrap_angle(candidate.yaw-prior.yaw))>o.max_rotation) {
      damping*=10;continue;
    }
    const double candidate_cost=tracking_objective(field,scan,candidate,prior,o);
    if (candidate_cost<cost) {
      pose=candidate;cost=candidate_cost;damping=std::max(1e-8,damping*0.3);
      if (std::hypot(step[0],step[1])<1e-4 && std::abs(step[2])<1e-5) break;
    } else damping*=10;
  }
  result.pose=pose;result.final_cost=cost;result.score=tracking_score(field,scan,pose,o);
  result.accepted=std::isfinite(cost) && result.score.inliers>=o.min_points && result.score.overlap>=o.min_overlap;
  if (!result.accepted) {
    // The caller retains the prediction on rejection. Scores must describe that
    // returned pose: recovery compares against this likelihood, and diagnostics
    // report this overlap. The discarded optimizer pose is not the live state.
    result.pose=prior;
    result.score=tracking_score(field,scan,prior,o);
    result.final_cost=result.initial_cost;
  }
  return result;
}

inline ScanPoints select_tracking_points(const ScanPoints& scan,std::size_t limit) {
  ScanPoints result;
  const auto count=std::min(scan.size(),limit);
  result.reserve(count);
  for (std::size_t i=0;i<count;++i) result.push_back(scan[i*scan.size()/count]);
  return result;
}

struct RecoveryOptions {
  bool enabled = true;
  double translation_window = 1.0, rotation_window = 0.35;
  double min_overlap = 0.55, ambiguity_margin = 0.05;
  std::size_t after_failures = 1, interval = 3, confirmations = 2;
};

// Recovery is deliberately separate from ordinary tracking. It searches a bounded
// region with weaker odometry regularization, then checks competing separated modes.
// The caller must corroborate the result on a later scan before inserting anything.
inline TrackingResult recover_tracking_scan(const TrackingField& field,const ScanPoints& scan,
                                            const PoseSample2& prior,const TrackingOptions& normal,
                                            const RecoveryOptions& recovery) {
  TrackingOptions o=normal;
  o.max_translation=recovery.translation_window; o.max_rotation=recovery.rotation_window;
  o.prior_translation_sigma=std::max(o.prior_translation_sigma,o.max_translation);
  o.prior_rotation_sigma=std::max(o.prior_rotation_sigma,o.max_rotation);
  o.min_overlap=std::max(o.min_overlap,recovery.min_overlap);
  const auto coarse=select_tracking_points(scan,90);
  struct Seed { PoseSample2 pose; double cost; };
  std::vector<Seed> seeds;
  // Eleven positions on each axis: fixed maximum work independent of trajectory size.
  for (int ix=-5;ix<=5;++ix) for (int iy=-5;iy<=5;++iy) for (int ia=-5;ia<=5;++ia) {
    const double dx=ix*o.max_translation/5,dy=iy*o.max_translation/5;
    if (std::hypot(dx,dy)>o.max_translation) continue;
    PoseSample2 p{prior.x+dx,prior.y+dy,wrap_angle(prior.yaw+ia*o.max_rotation/5)};
    seeds.push_back({p,tracking_objective(field,coarse,p,prior,o)});
  }
  std::stable_sort(seeds.begin(),seeds.end(),[](const auto& a,const auto& b) {return a.cost<b.cost;});
  std::vector<PoseSample2> refined_seeds;
  std::vector<TrackingResult> matches;
  for (const auto& seed:seeds) {
    bool distinct=true;
    for (const auto& other:refined_seeds)
      if (std::hypot(seed.pose.x-other.x,seed.pose.y-other.y)<0.30 &&
          std::abs(wrap_angle(seed.pose.yaw-other.yaw))<0.12) distinct=false;
    if (!distinct) continue;
    refined_seeds.push_back(seed.pose);
    auto match=match_tracking_scan(field,scan,prior,o,&seed.pose);
    if (match.accepted) matches.push_back(match);
    if (refined_seeds.size()==6) break;
  }
  TrackingResult failed;failed.pose=prior;
  if (matches.empty()) return failed;
  std::stable_sort(matches.begin(),matches.end(),[](const auto& a,const auto& b) {return a.final_cost<b.final_cost;});
  auto best=matches.front();
  if (best.initial_cost-best.final_cost<0.05) return failed;
  for (std::size_t i=1;i<matches.size();++i) {
    const auto& other=matches[i];
    const bool separated=std::hypot(best.pose.x-other.pose.x,best.pose.y-other.pose.y)>0.45 ||
        std::abs(wrap_angle(best.pose.yaw-other.pose.yaw))>0.18;
    if (separated && best.score.mean_log_likelihood-other.score.mean_log_likelihood<recovery.ambiguity_margin)
      return failed;
  }
  return best;
}

}  // namespace belugaslam
#endif
