#include "belugaslam_core/output_selection.hpp"
#include <algorithm>
#include <iostream>
#include <random>
#include <stdexcept>
using namespace belugaslam;
int checks=0;
void Check(bool condition,const char* message) {++checks;if(!condition)throw std::runtime_error(message);}
int main() {
  const std::vector<OutputPoseHypothesis> initial{{0,.4,0,0},{1,.6,1,0}};
  auto a=select_output_pose(initial);
  Check(initial[a.risk_index].x==1,"unsplit belief");
  const std::vector<OutputPoseHypothesis> split{{0,.4,0,0},{1,.3,1,0},{2,.3,1,0}};
  auto b=select_output_pose(split);
  Check(split[b.map_index].x==0,"reproduce MAP-ID fragmentation");
  Check(split[b.risk_index].x==1,"splitting identical pose mass changed position decision");
  Check(std::abs(b.minimum_position_risk-.4)<1e-12,"risk units/normalization");
  Check(std::abs(a.minimum_position_risk-b.minimum_position_risk)<1e-12,"risk changed under mass split");
  const auto two=select_output_pose({{0,.7,0,0},{1,.3,10,0}});
  Check(two.map_index==two.risk_index,"two-mode majority should remain selected");
  const auto zero=select_output_pose({{0,.5,-1,0},{1,.5,1,0},{2,0,0,0}});
  Check(zero.risk_index!=2,"zero-mass midpoint cannot be published");
  const auto single=select_output_pose({{10,3,8,-2}});
  Check(single.risk_index==0 && single.minimum_position_risk==0,"single hypothesis identity");
  for(const auto& bad:std::vector<std::vector<OutputPoseHypothesis>>{
      {},{{0,0,0,0}},{{0,-1,0,0}},{{0,1,std::numeric_limits<double>::infinity(),0}}}) {
    bool threw=false;try{(void)select_output_pose(bad);}catch(const std::invalid_argument&){threw=true;}
    Check(threw,"invalid beliefs must fail explicitly");
  }
  // Unique minimizers are invariant to rigid frame changes, normalization and order.
  std::mt19937 rng(42);std::uniform_real_distribution<double> draw(-10,10);
  for(int n=0;n<100;++n) {
    std::vector<OutputPoseHypothesis> h;
    for(std::size_t i=0;i<4;++i)h.push_back({i,1+std::abs(draw(rng)),draw(rng),draw(rng)});
    const auto before=select_output_pose(h);const auto winner=h[before.risk_index].id;
    Check(before.minimum_position_risk<=before.map_position_risk,"risk exceeds MAP candidate");
    const double angle=.73,c=std::cos(angle),s=std::sin(angle);
    for(auto& item:h){const double x=item.x,y=item.y;item.x=100+c*x-s*y;item.y=-50+s*x+c*y;item.mass*=7;}
    std::reverse(h.begin(),h.end());const auto after=select_output_pose(h);
    Check(h[after.risk_index].id==winner,"decision depends on coordinate frame/order/mass scale");
    Check(std::abs(before.minimum_position_risk-after.minimum_position_risk)<1e-10,"risk depends on rigid frame");
  }
  // Actual first-loop switch snapshot (scan 1694): .605 mass near the same pose.
  const std::vector<OutputPoseHypothesis> logged{
    {11,.39474595924135814,-5.1126837116579171,.53745775492085635},
    {14,.22449404081961716,-6.5231805437691541,-.19573410507517197},
    {15,.38075999993902471,-6.5231913138174829,-.19566214514381475}};
  const auto observed=select_output_pose(logged);
  Check(logged[observed.map_index].id==11,"logged MAP mode");
  Check(logged[observed.risk_index].id!=11,"logged support fragmentation not resolved");
  Check(observed.minimum_position_risk<observed.map_position_risk,"logged loss did not improve");
  std::cout<<"PASS: "<<checks<<" output-selection checks\n";
}
