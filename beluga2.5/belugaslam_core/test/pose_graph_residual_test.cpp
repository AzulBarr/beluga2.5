#include "belugaslam_core/pose_graph_residual.hpp"
#include <array>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

using Pose=std::array<double,3>;
void require(bool good,const char* message) {if(!good)throw std::runtime_error(message);}
// Direct transform construction used by the previous AutoDiff implementation.
Pose reference(const Pose& a,const Pose& b,const std::array<double,8>& m) {
  const double x=a[0]+std::cos(a[2])*m[5]-std::sin(a[2])*m[6];
  const double y=a[1]+std::sin(a[2])*m[5]+std::cos(a[2])*m[6];
  const double t=a[2]+m[7],c=std::cos(t),s=std::sin(t),da=b[2]-t-m[2];
  return {(c*(b[0]-x)+s*(b[1]-y)-m[0])*m[3],
          (-s*(b[0]-x)+c*(b[1]-y)-m[1])*m[3],
          std::atan2(std::sin(da),std::cos(da))*m[4]};
}
int main() {
  std::mt19937 rng(42);std::uniform_real_distribution<double> draw(-20,20),angle(-2,2),weight(.2,20);
  double max_value_error=0,max_derivative_error=0;
  for(int n=0;n<2000;++n) {
    Pose a{draw(rng),draw(rng),angle(rng)},b{draw(rng),draw(rng),0};
    std::array<double,8> m{draw(rng),draw(rng),angle(rng),weight(rng),weight(rng),draw(rng),draw(rng),angle(rng)};
    b[2]=a[2]+m[7]+m[2]+angle(rng); // away from the angular branch cut
    belugaslam::PoseGraphResidual cost(m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7]);
    double r[3],ja[9],jb[9];require(cost.evaluate(a.data(),b.data(),r,ja,jb),"finite residual");
    const auto old=reference(a,b,m);
    for(int k=0;k<3;++k)max_value_error=std::max(max_value_error,std::abs(r[k]-old[k]));
    for(int block=0;block<2;++block)for(int column=0;column<3;++column) {
      auto ap=a,am=a,bp=b,bm=b;constexpr double step=1e-6;
      (block==0?ap:bp)[column]+=step;(block==0?am:bm)[column]-=step;
      const auto plus=reference(ap,bp,m),minus=reference(am,bm,m);
      for(int row=0;row<3;++row) {
        const double fd=(plus[row]-minus[row])/(2*step),actual=(block==0?ja:jb)[row*3+column];
        max_derivative_error=std::max(max_derivative_error,std::abs(fd-actual));
      }
    }
    double values_only[3],partial[9];require(cost.evaluate(a.data(),b.data(),values_only),"values-only evaluation");
    require(cost.evaluate(a.data(),b.data(),values_only,nullptr,partial),"constant first block");
    for(int i=0;i<9;++i)require(partial[i]==jb[i],"partial Jacobian layout");
    require(cost.evaluate(a.data(),b.data(),values_only,partial,nullptr),"constant second block");
    for(int i=0;i<9;++i)require(partial[i]==ja[i],"partial Jacobian layout");
  }
  require(max_value_error<1e-10,"changed residual objective");
  require(max_derivative_error<1e-5,"analytic derivative differs from finite differences");
  belugaslam::PoseGraphResidual identity(0,0,0);
  Pose a{0,0,3.13},b{0,0,-3.13};double r[3];
  require(identity.evaluate(a.data(),b.data(),r)&&std::abs(r[2]-.023185307179586)<1e-12,"angle wrap");
  b[0]=std::numeric_limits<double>::infinity();require(!identity.evaluate(a.data(),b.data(),r),"nonfinite input");
  require(belugaslam::weighted_loop_residual_squared(.08,.02,10,12)<1,"quadratic-region fast path");
  require(belugaslam::weighted_loop_residual_squared(.2,.02,10,12)>1,"robust-region check");
  std::cout<<"PASS: 2000 residual/Jacobian fixtures, max residual error="<<max_value_error
           <<", max finite-difference error="<<max_derivative_error<<"\n";
}
