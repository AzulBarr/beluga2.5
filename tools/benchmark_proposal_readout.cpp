// Isolated serial component timing; NOT full SLAM or real-time performance.
#include "belugaslam_core/proposal_pose.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
using namespace belugaslam;
int main() {
  std::vector<float> cells(240*240,0);ScanPoints scan;
  for(int i=0;i<180;++i) {cells[(30+i)*240+160]=5;scan.emplace_back(2.025,-6+(30+i+.5)*.05);}
  TrackingField field(cells,240,240,.05,-6,-6);TrackingOptions options;
  std::mt19937 rng(42);std::normal_distribution<double> normal(0,.03);
  volatile double checksum=0;
  std::cout<<"particles,proposals,mean_scoring_ms,mean_readout_ms\n"<<std::setprecision(8);
  for(int particles:{30,300,1000}) {
    std::vector<WeightedPoseProposal> cloud;
    for(int i=0;i<particles*8;++i)cloud.push_back({{normal(rng),normal(rng),.2*normal(rng)},0});
    double scoring=0,readout=0;
    for(int iteration=0;iteration<50;++iteration) {
      cloud.front().pose.x+=1e-8;
      const auto begin=std::chrono::steady_clock::now();
      for(auto& p:cloud)p.log_weight=options.effective_beams*tracking_score(field,scan,p.pose,options).mean_log_likelihood;
      const auto scored=std::chrono::steady_clock::now();
      const auto weights=normalized_proposal_weights(cloud);
      const auto summary=summarize_proposal_poses(cloud,weights,{},.5,.25);
      const auto decision=check_proposal_pose(summary,field,scan,{},tracking_score(field,scan,{},options),options);
      const auto second=proposal_second_moment(cloud,weights,{});
      checksum=checksum+summary.mean.x+second[0]+decision.score.overlap;
      const auto end=std::chrono::steady_clock::now();
      scoring+=std::chrono::duration<double,std::milli>(scored-begin).count();
      readout+=std::chrono::duration<double,std::milli>(end-scored).count();
    }
    std::cout<<particles<<','<<cloud.size()<<','<<scoring/50<<','<<readout/50<<'\n';
  }
  return std::isfinite(checksum) ? 0 : 1;
}
