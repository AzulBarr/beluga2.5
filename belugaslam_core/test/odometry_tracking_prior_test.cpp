#include "belugaslam_core/probability_matching.hpp"
#include <iostream>
#include <random>
#include <stdexcept>
using namespace belugaslam;
int checks=0;
void check(bool yes,const char* message) {++checks;if(!yes)throw std::runtime_error(message);}
PriorMatrix3 multiply(const PriorMatrix3& a,const PriorMatrix3& b) {
  PriorMatrix3 c{};
  for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k)c[3*i+j]+=a[3*i+k]*b[3*k+j];
  return c;
}
PriorMatrix3 transpose(PriorMatrix3 a) {
  for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)std::swap(a[3*i+j],a[3*j+i]);
  return a;
}
int main() {
  OdometryPriorOptions options;
  const auto rest=odometry_tracking_prior({0,0,0},1.1,options);
  check(std::abs(rest.covariance[0]-.01)<1e-14 && std::abs(rest.covariance[8]-.0025)<1e-14,
        "stationary covariance must equal configured nonzero floor");
  check(rest.covariance[1]==0 && rest.covariance[2]==0 && rest.covariance[5]==0,"stationary cross covariance");
  const auto turn=odometry_tracking_prior({0,0,.3},0,options);
  check(turn.covariance[8]>rest.covariance[8],"rotation must increase yaw uncertainty");
  const auto half_turn=odometry_tracking_prior({0,0,3.141592653589793},0,options);
  check(half_turn.covariance[8]>.5,"pure half turn must not be mistaken for backward translation");
  const auto forward=odometry_tracking_prior({.4,0,0},0,options);
  const auto backward=odometry_tracking_prior({-.4,0,0},0,options);
  for(int i:{0,4,8})check(std::abs(forward.covariance[i]-backward.covariance[i])<1e-12,"backward motion got artificial pi noise");
  check(forward.covariance[5]>0 && backward.covariance[5]<0,"translation/yaw correlation sign");
  check(forward.covariance[0]>rest.covariance[0],"travel must increase translation uncertainty");
  const PoseSample2 motion{.32,.04,.07};
  const auto base=odometry_tracking_prior(motion,.2,options);
  const double angle=.81,c=std::cos(angle),s=std::sin(angle);
  const PriorMatrix3 rotation{c,-s,0,s,c,0,0,0,1};
  const auto rotated=odometry_tracking_prior(motion,.2+angle,options);
  const auto expected=multiply(multiply(rotation,base.covariance),transpose(rotation));
  for(int i=0;i<9;++i)check(std::abs(rotated.covariance[i]-expected[i])<1e-12,"submap-frame covariance transport");
  for(const auto& prior:{rest,turn,forward,backward,base,rotated}) {
    const auto identity=multiply(multiply(prior.sqrt_information,prior.covariance),transpose(prior.sqrt_information));
    for(int i=0;i<9;++i)check(std::abs(identity[i]-(i%4==0?1.:0.))<1e-10,"whitening factor orientation");
  }
  // Full covariance must enter BOTH objectives with the same Mahalanobis cost.
  std::vector<float> cells(40*40,-.4F);
  for(int y=0;y<40;++y)for(int x=0;x<40;++x)cells[y*40+x]=float(.5+.4*std::sin(.3*x)+.2*std::cos(.4*y));
  ProbabilityField probability(cells,40,40,.1,-2,-2);
  TrackingField field(cells,40,40,.1,-2,-2);
  TrackingOptions tracking;tracking.use_full_prior=true;tracking.prior_sqrt_information=base.sqrt_information;
  ProbabilityMatchingOptions p_options;
  const ScanPoints scan{{.2,.3},{.7,-.4},{-.2,.7},{-.6,-.3}};
  const PoseSample2 prior{.03,-.02,3.13};
  double delta[]{.015,-.017,.025};
  const auto error=full_tracking_prior_residual(delta,tracking);
  double quadratic=0;for(double v:error)quadratic+=.5*v*v;
  std::array<std::array<double,3>,3> sigma{};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)sigma[i][j]=base.covariance[3*i+j];
  std::array<double,3> solved{};
  check(solve_tracking_system(sigma,{delta[0],delta[1],delta[2]},solved),"independent covariance solve");
  check(std::abs(quadratic-.5*(delta[0]*solved[0]+delta[1]*solved[1]+delta[2]*solved[2]))<1e-12,"Mahalanobis cost");
  const PoseSample2 candidate{prior.x+delta[0],prior.y+delta[1],wrap_angle(prior.yaw+delta[2])};
  check(std::abs(tracking_objective(field,scan,candidate,prior,tracking)+
        tracking_score(field,scan,candidate,tracking).mean_log_likelihood-quadratic)<1e-12,"distance objective ignores covariance");
  std::vector<double> residuals(scan.size()+3),jacobian(3*residuals.size()),plus(residuals.size()),minus(residuals.size());
  check(evaluate_probability_match(probability,scan,prior,tracking,p_options,delta,residuals.data(),jacobian.data()),"Ceres residual kernel");
  for(int a=0;a<3;++a)check(std::abs(residuals[scan.size()+a]-error[a])<1e-12,"Ceres and distance priors differ");
  for(int a=0;a<3;++a) {
    const double eps=1e-6;delta[a]+=eps;
    evaluate_probability_match(probability,scan,prior,tracking,p_options,delta,plus.data());
    delta[a]-=2*eps;
    evaluate_probability_match(probability,scan,prior,tracking,p_options,delta,minus.data());delta[a]+=eps;
    for(std::size_t i=0;i<residuals.size();++i)
      check(std::abs(jacobian[3*i+a]-(plus[i]-minus[i])/(2*eps))<2e-6,"full-covariance residual Jacobian");
  }
  auto diagonal=tracking;diagonal.prior_sqrt_information=fixed_tracking_prior(.5,.2).sqrt_information;
  auto legacy=tracking;legacy.use_full_prior=false;
  check(std::abs(tracking_objective(field,scan,candidate,prior,diagonal)-tracking_objective(field,scan,candidate,prior,legacy))<1e-12,
        "diagonal full prior is not equivalent to legacy prior");
  // Monte Carlo through the nonlinear motion model, independent of the matrix
  // implementation. Small-noise covariance should agree with its linearization.
  auto mc=options;mc.translation_sigma=.02;mc.rotation_sigma=.01;
  mc.alpha1=.002;mc.alpha2=.001;mc.alpha3=.003;mc.alpha4=.001;
  const PoseSample2 step{.4,.08,.12};const double previous=.4;
  const auto analytic=odometry_tracking_prior(step,previous,mc);
  const double distance=std::hypot(step.x,step.y),r1=std::atan2(step.y,step.x),r2=step.yaw-r1;
  const double sr1=std::sqrt(mc.alpha1*r1*r1+mc.alpha2*distance*distance);
  const double st=std::sqrt(mc.alpha3*distance*distance+mc.alpha4*(r1*r1+r2*r2));
  const double sr2=std::sqrt(mc.alpha1*r2*r2+mc.alpha2*distance*distance);
  std::mt19937 engine(1729);std::normal_distribution<double> normal;
  std::array<double,3> mean{};PriorMatrix3 second{};
  constexpr int count=80000;
  for(int i=0;i<count;++i) {
    const double px=mc.translation_sigma*normal(engine),py=mc.translation_sigma*normal(engine);
    const double yaw=previous+mc.rotation_sigma*normal(engine);
    const double a=r1+sr1*normal(engine),d=distance+st*normal(engine),b=r2+sr2*normal(engine);
    const std::array<double,3> e{px+d*std::cos(yaw+a)-distance*std::cos(previous+r1),
                                py+d*std::sin(yaw+a)-distance*std::sin(previous+r1),
                                yaw+a+b-(previous+step.yaw)};
    for(int j=0;j<3;++j){mean[j]+=e[j]/count;for(int k=0;k<3;++k)second[3*j+k]+=e[j]*e[k]/count;}
  }
  double norm_error=0,norm_reference=0;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
    const double diff=second[3*i+j]-mean[i]*mean[j]-analytic.covariance[3*i+j];
    norm_error+=diff*diff;norm_reference+=analytic.covariance[3*i+j]*analytic.covariance[3*i+j];
  }
  check(std::sqrt(norm_error/norm_reference)<.025,"nonlinear Monte Carlo covariance mismatch");
  for(double invalid:{-1.,double(NAN),double(INFINITY)}) {
    bool threw=false;try{auto o=options;o.translation_sigma=invalid;o.validate();}catch(const std::invalid_argument&){threw=true;}
    check(threw,"invalid prior sigma accepted");
  }
  bool threw=false;auto bad=base.covariance;bad[1]=NAN;
  try{factor_tracking_prior(bad);}catch(const std::invalid_argument&){threw=true;}
  check(threw,"non-finite upper covariance accepted");
  std::cout<<"PASS: "<<checks<<" odometry prior checks; nonlinear covariance relative error="<<std::sqrt(norm_error/norm_reference)<<'\n';
}
