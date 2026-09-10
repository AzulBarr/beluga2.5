// Test bridge for a native Ceres solver exposed through Python bindings.
// It invokes the same compiled C++ residual/Jacobian kernel as production.
#include "belugaslam_core/pose_graph_residual.hpp"
extern "C" int beluga_pose_graph_evaluate(const double* m,const double* a,const double* b,
                                          double* r,double* ja,double* jb) {
  return belugaslam::PoseGraphResidual(m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7]).evaluate(a,b,r,ja,jb);
}
