#include "belugaslam_core/scan_informed_proposal.hpp"
#include "belugaslam_core/particle_proposal.hpp"
#include "belugaslam_core/hierarchical_bayes.hpp"
#include <iostream>
#include <iomanip>

using namespace belugaslam;
namespace {
int checks=0;
void require(bool ok,const char* message) {
  ++checks;if(!ok) throw std::runtime_error(message);
}
void near(double x,double y,double tolerance,const char* message) {
  if(std::abs(x-y)>tolerance) std::cerr<<message<<": "<<x<<" vs "<<y<<'\n';
  require(std::isfinite(x)&&std::abs(x-y)<=tolerance,message);
}
double likelihood(const MotionIncrement& d) {
  return .05+.95*std::exp(-.5*std::pow((d[0]-.68)/.09,2)-.5*std::pow((d[1]-.06)/.12,2));
}
}
int main() {
  using namespace beluga::motion_detail;
  const beluga::DifferentialDriveModelParam params{.1,.05,.1,.05,.01};
  const auto prior=beluga::make_differential_drive_distribution(.6,.1,.1,.2,params);
  require(prior.has_density(),"moving prior has a density");
  require(!beluga::make_differential_drive_distribution(0,0,0,0,params).has_density(),"stationary prior is singular");
  require(!beluga::make_differential_drive_distribution(.2,0,0,0,{0,0,0,0,.01}).has_density(),"zero noise is singular");
  require(!beluga::make_differential_drive_distribution(0,0,.3,.3,params).has_density(),"pure rotation retains singular first heading");
  const auto reverse=beluga::make_differential_drive_distribution(.005,pi,-pi,0,params);
  near(reverse.first_mean,pi,0,"small reverse step stays reverse");
  near(reverse.first_sigma,std::sqrt(.05*.005*.005),1e-15,"threshold affects noise, not displacement");

  for(double sigma : {.02,.3,.99,1.,2.,10.}) {
    double integral=0;const int steps=20000;
    for(int i=0;i<steps;++i) integral+=std::exp(wrapped_normal_log_density(-pi+(i+.5)*2*pi/steps,pi-.01,sigma))*2*pi/steps;
    near(integral,1,2e-12,"wrapped normal integrates to one");
    near(wrapped_normal_log_density(-pi+.005,pi-.01,sigma),
         wrapped_normal_log_density(pi+.005,pi-.01,sigma),1e-9,"angle seam density");
  }
  // Explicit independent preimage calculation: large signed-distance noise gives
  // appreciable mass to BOTH polar representations of exactly the same pose.
  const beluga::DifferentialDriveDistribution2d broad{.2,.1,.1,.4,.5,.6};
  const double radius=.3,bearing=.25,yaw=.1;
  const auto wrapped=[&](double x,double mean,double sigma) {
    double sum=0;for(int k=-8;k<=8;++k) sum+=std::exp(normal_log_density(x+2*pi*k,mean,sigma));return sum;
  };
  const auto branch=[&](double r,double a) {
    return std::exp(normal_log_density(r,broad.distance_mean,broad.distance_sigma))*
      wrapped(a,broad.first_mean,broad.first_sigma)*wrapped(yaw-a,broad.second_mean,broad.second_sigma);
  };
  near(std::exp(broad.log_density(radius*std::cos(bearing),radius*std::sin(bearing),yaw)),
       (branch(radius,bearing)+branch(-radius,bearing+pi))/radius,1e-12,"signed distance branches and Jacobian");
  near(broad.log_density(.2,.1,yaw),broad.log_density(.2,.1,yaw+2*pi),1e-12,"motion density wraps yaw");

  // Numerical change-of-measure checks compare independently drawn prior and q.
  // With L=1, correcting a scan-guided q must recover the ORIGINAL prior, not
  // stay concentrated at the scan pose (the double-counting regression).
  std::mt19937 rng(9123);
  for(const auto& d : std::vector<beluga::DifferentialDriveDistribution2d>{prior,broad,
      beluga::make_differential_drive_distribution(.4,pi,-pi+.2,.2,params)}) {
    const MotionIncrement frontend{d.distance_mean*std::cos(d.first_mean)+.07,
      d.distance_mean*std::sin(d.first_mean)+.04,wrap(d.first_mean+d.second_mean+.04)};
    ScanProposalOptions o;o.adapt_to_prior=false;
    ScanInformedProposal q(d,frontend,o);
    constexpr int N=240000;
    double norm=0,mx=0,my=0,cyaw=0,evidence=0,weighted_x=0;
    double px=0,py=0,pc=0,pe=0,pex=0;
    for(int i=0;i<N;++i) {
      const auto p=d.sample(rng);const auto draw=q.draw(d.sample(rng),rng);
      const double w=std::exp(draw.log_ratio),l=likelihood(draw.delta);
      require(draw.log_ratio<=-std::log1p(-o.fraction)+1e-12,"defensive mixture bounds p/q without clipping");
      norm+=w;mx+=w*draw.delta[0];my+=w*draw.delta[1];cyaw+=w*std::cos(draw.delta[2]);
      evidence+=w*l;weighted_x+=w*l*draw.delta[0];
      px+=p[0];py+=p[1];pc+=std::cos(p[2]);pe+=likelihood(p);pex+=likelihood(p)*p[0];
    }
    near(norm/N,1,.012,"E_q[p/q]=1");
    near(mx/N,px/N,.008,"corrected x moment recovers prior");
    near(my/N,py/N,.008,"corrected y moment recovers prior");
    near(cyaw/N,pc/N,.012,"corrected circular yaw moment");
    near(evidence/N,pe/N,.008,"E_q[L p/q] equals predictive evidence");
    near(weighted_x/evidence,pex/pe,.008,"corrected posterior readout");
  }
  const MotionIncrement odom{prior.distance_mean*std::cos(prior.first_mean),
    prior.distance_mean*std::sin(prior.first_mean),wrap(prior.first_mean+prior.second_mean)};
  const MotionIncrement front{.65,.1,.25};
  ScanInformedProposal transported(prior,front,ScanProposalOptions{});
  for(int i=0;i<100;++i) {
    const auto raw=prior.sample(rng);const auto d=transported.draw(raw,rng);
    const auto expected=d.from_frontend ? compose_increment(front,compose_increment(inverse_increment(odom),raw)) : raw;
    for(int axis=0;axis<3;++axis) near(d.delta[axis],expected[axis],1e-12,"frontend then original right-composed noise");
  }
  ScanInformedProposal same(prior,odom,ScanProposalOptions{});
  near(same.fraction(),.8,1e-14,"compatible frontend retains configured mixture");
  ScanInformedProposal unsupported(prior,{10,10,2},ScanProposalOptions{});
  require(unsupported.fraction()<1e-20,"unsupported frontend cannot waste most candidates");
  ScanInformedProposal adaptive(prior,{.67,.1,.24},ScanProposalOptions{});
  require(adaptive.fraction()>0 && adaptive.fraction()<.8,"soft support adaptation");
  double adaptive_norm=0;
  for(int i=0;i<180000;++i) adaptive_norm+=std::exp(adaptive.draw(prior.sample(rng),rng).log_ratio);
  near(adaptive_norm/180000,1,.012,"adaptive coefficient is also used in denominator q");
  ScanProposalOptions pure_options;pure_options.fraction=1.;pure_options.adapt_to_prior=false;
  ScanInformedProposal pure(prior,compose_increment(odom,{.001,.001,.001}),pure_options);
  double pure_norm=0;
  for(int i=0;i<180000;++i) {
    const auto raw=prior.sample(rng);const auto before=rng;const auto d=pure.draw(raw,rng);
    require(d.from_frontend && rng==before,"pure proposal always transforms; no mixture draw");
    near(d.log_ratio,prior.log_density(d.delta[0],d.delta[1],d.delta[2])-pure.log_frontend_density(d.delta),
      1e-12,"pure correction uses exactly p/q_frontend");
    pure_norm+=std::exp(d.log_ratio);
  }
  near(pure_norm/180000,1.,.012,"pure proposal change of measure");
  ScanProposalOptions off;off.fraction=0;
  ScanInformedProposal qoff(prior,{2,3,.4},off);
  const auto generator_before=rng;
  const MotionIncrement candidate{.4,.1,.2};const auto draw=qoff.draw(candidate,rng);
  require(rng==generator_before && draw.delta==candidate && draw.log_ratio==0 && !draw.from_frontend,
      "disabled proposal retains draws and RNG");
  bool rejected=false;try {ScanProposalOptions bad;bad.fraction=1.01;bad.validate();}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"cannot exceed probability one");
  const auto choice=select_motion_proposal({-INFINITY,std::log(.4)},rng);
  require(choice.index==1,"zero target density is never selected");
  near(std::exp(choice.log_evidence),.2,1e-14,"zero-weight candidates remain in K normalizer");

  // Multi-try selection is unbiased only if the ancestor receives the MEAN
  // corrected factor, including the candidate count and the full mixture q.
  ScanProposalOptions options;options.adapt_to_prior=false;
  ScanInformedProposal q(prior,{.67,.08,.22},options);
  double evidence=0,selected_moment=0,reference_evidence=0,reference_moment=0;
  constexpr int trials=45000,K=4;
  for(int i=0;i<trials;++i) {
    std::vector<MotionIncrement> candidates;std::vector<double> logs;
    for(int k=0;k<K;++k) {
      const auto d=q.draw(prior.sample(rng),rng);candidates.push_back(d.delta);
      logs.push_back(d.log_ratio+std::log(likelihood(d.delta)));
      const auto p=prior.sample(rng);reference_evidence+=likelihood(p)/K;reference_moment+=likelihood(p)*p[0]/K;
    }
    const auto pick=select_motion_proposal(logs,rng);const double w=std::exp(pick.log_evidence);
    evidence+=w;selected_moment+=w*candidates[pick.index][0];
  }
  near(evidence/trials,reference_evidence/trials,.01,"multi-try predictive evidence");
  near(selected_moment/trials,reference_moment/trials,.008,"multi-try selected weighted moment");
  std::cout<<"PASS: "<<checks<<" scan-informed checks (720000 change-of-measure samples, 45000 multi-try trials)\n";
}
