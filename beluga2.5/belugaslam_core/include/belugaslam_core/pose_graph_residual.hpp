#ifndef BELUGASLAM_CORE_POSE_GRAPH_RESIDUAL_HPP
#define BELUGASLAM_CORE_POSE_GRAPH_RESIDUAL_HPP
#include <algorithm>
#include <cmath>

namespace belugaslam {
// The same three weighted residuals as PoseGraphEdgeError, with explicit
// row-major derivatives for both [x,y,yaw] blocks, including a rigid offset.
class PoseGraphResidual {
public:
  PoseGraphResidual(double dx, double dy, double angle, double wt=1, double wr=1,
                    double ox=0, double oy=0, double oa=0)
      : dx_(dx), dy_(dy), angle_(angle), wt_(wt), wr_(wr), oa_(oa),
        offset_x_(std::cos(oa)*ox+std::sin(oa)*oy),
        offset_y_(-std::sin(oa)*ox+std::cos(oa)*oy) {}

  bool evaluate(const double* a, const double* b, double* r,
                double* ja=nullptr, double* jb=nullptr) const {
    const double theta=a[2]+oa_, c=std::cos(theta), s=std::sin(theta);
    const double x=b[0]-a[0], y=b[1]-a[1];
    const double u=c*x+s*y, v=-s*x+c*y;
    const double da=b[2]-theta-angle_;
    r[0]=wt_*(u-offset_x_-dx_);
    r[1]=wt_*(v-offset_y_-dy_);
    r[2]=wr_*std::atan2(std::sin(da),std::cos(da));
    if (!std::isfinite(r[0]) || !std::isfinite(r[1]) || !std::isfinite(r[2])) return false;
    if (ja) {
      const double values[9]={-wt_*c,-wt_*s,wt_*v,
                               wt_*s,-wt_*c,-wt_*u, 0,0,-wr_};
      std::copy(values,values+9,ja);
    }
    if (jb) {
      const double values[9]={wt_*c,wt_*s,0, -wt_*s,wt_*c,0, 0,0,wr_};
      std::copy(values,values+9,jb);
    }
    return true;
  }
private:
  double dx_,dy_,angle_,wt_,wr_,oa_,offset_x_,offset_y_;
};

inline double weighted_loop_residual_squared(double translation_error, double rotation_error,
                                             double translation_weight, double rotation_weight) {
  const double t=translation_weight*translation_error, a=rotation_weight*rotation_error;
  return t*t+a*a;
}
}  // namespace belugaslam
#endif
