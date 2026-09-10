#include "belugaslam_core/loop_search.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <utility>
using namespace belugaslam;
namespace {
int checks=0;
void check(bool ok,const char* why) {++checks;if (!ok) throw std::runtime_error(why);}
PoseSample2 compose(PoseSample2 a,PoseSample2 b) {
  const double c=std::cos(a.yaw),s=std::sin(a.yaw);
  return {a.x+c*b.x-s*b.y,a.y+s*b.x+c*b.y,wrap_angle(a.yaw+b.yaw)};
}
// The previous delivered coarse/beam schedule, retained here only as a regression
// comparator. It uses the same score callback to isolate search from rasterization.
template<class Score> LoopSearchMatch previous_search(Score score) {
  std::vector<LoopSearchMatch> beam;
  for(double x=-3;x<=3+1e-9;x+=.5) for(double y=-3;y<=3+1e-9;y+=.5)
    for(double a=-.7;a<=.7+1e-9;a+=.14) {PoseSample2 p{x,y,a};beam.push_back({p,score(p)});}
  auto keep=[](auto& states) {
    std::stable_sort(states.begin(),states.end(),[](const auto& a,const auto& b) {return a.value.score>b.value.score;});
    if(states.size()>8) states.resize(8);
  };
  keep(beam);
  for(const auto& step:std::vector<std::pair<double,double>>{{.15,.04},{.05,.015},{.015,.005}}) {
    std::vector<LoopSearchMatch> refined;
    for(const auto& seed:beam) for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y) for(int a=-1;a<=1;++a) {
      const auto p=compose(seed.pose,{x*step.first,y*step.first,a*step.second});
      refined.push_back({p,score(p)});
    }
    keep(refined);beam=std::move(refined);
  }
  return beam.front();
}
}
int main() {
  std::vector<float> distances{0,.1F,.2F,.3F};std::vector<double> values{1,.8,.6,.4};
  LoopFieldView small(distances,values,2,2,.1,0,0);
  check(std::abs(small.sample(.05,.05).score-1)<1e-12,"cell center moved");
  check(std::abs(small.sample(.1,.1).score-.7)<1e-12,"bilinear score");
  check(std::abs(small.sample(.1,.1).distance-.15)<1e-8,"bilinear distance");
  check(small.sample(-100,0).score==0,"outside score");
  check(small.sample(NAN,0).score==0,"nan coordinate");
  bool threw=false;try {LoopFieldView bad(distances,values,1,2,.1,0,0);}catch(const std::invalid_argument&){threw=true;}
  check(threw,"invalid extent");
  LoopSearchOptions o;
  const PoseSample2 truth{.25,-.25,.07};
  auto peak=[&](const PoseSample2& p) {return LoopScore{std::exp(-.5*(
      std::pow((p.x-truth.x)/.4,2)+std::pow((p.y-truth.y)/.4,2)+std::pow(wrap_angle(p.yaw-truth.yaw)/.2,2))),1};};
  const auto old=previous_search(peak);const auto now=search_loop_modes({0,0,0},o,peak);
  check(!now.empty(),"coarse-gap target lost");
  const double old_error=std::hypot(old.pose.x-truth.x,old.pose.y-truth.y);
  const double new_error=std::hypot(now[0].pose.x-truth.x,now[0].pose.y-truth.y);
  check(new_error<.006,"halving schedule leaves a gap");
  check(std::abs(wrap_angle(now[0].pose.yaw-truth.yaw))<.002,"angular refinement gap");
  check(new_error<old_error,"search gap regression not improved");
  std::cout<<"Coarse-gap synthetic translation error: previous="<<old_error<<" new="<<new_error<<'\n';
  auto twin=[](const PoseSample2& p) {
    const double dx=std::min(std::abs(p.x-1),std::abs(p.x+1));
    return LoopScore{std::exp(-.5*(dx*dx/.04+p.y*p.y/.04+p.yaw*p.yaw/.01)),1};
  };
  const auto twins=search_loop_modes({0,0,0},o,twin);
  check(twins.size()==2,"distinct alternatives collapsed");
  check(twins[0].pose.x*twins[1].pose.x<0,"same basin returned twice");
  o.max_modes=1;check(search_loop_modes({0,0,0},o,twin).size()==1,"mode ablation limit");o.max_modes=2;
  const auto corridor=search_loop_modes({0,0,0},o,[](const auto& p) {
    return LoopScore{std::exp(-p.x*p.x/.04-p.yaw*p.yaw/.01),1};
  });
  check(!corridor.empty() && corridor.front().pose.y==0,"flat direction moved to window boundary");
  auto again=search_loop_modes({0,0,0},o,twin);
  check(again[0].pose.x==twins[0].pose.x && again[1].pose.x==twins[1].pose.x,"search not deterministic");
  check(search_loop_modes({0,0,0},o,[](const auto&){return LoopScore{};}).empty(),"unsupported match");
  check(search_loop_modes({0,0,0},o,[](const auto&){return LoopScore{NAN,1};}).empty(),"nonfinite match");
  check(search_loop_modes({INFINITY,0,0},o,peak).empty(),"invalid initial pose");
  o.translation_window=0;o.rotation_window=0;int calls=0;
  const auto zero=search_loop_modes(truth,o,[&](const auto& p){++calls;return peak(p);});
  check(calls==1 && zero.size()==1,"zero window excludes initial or repeats work");
  check(zero[0].pose.x==truth.x,"zero window moved pose");
  o.translation_window=.07;o.rotation_window=.03;
  const PoseSample2 origin{2,4,1.2};const double c=std::cos(origin.yaw),s=std::sin(origin.yaw);
  bool bounded=true;
  (void)search_loop_modes(origin,o,[&](const auto& p) {
    const double x=p.x-origin.x,y=p.y-origin.y;
    bounded=bounded&&std::abs(c*x+s*y)<=.07000001&&std::abs(-s*x+c*y)<=.07000001&&
        std::abs(wrap_angle(p.yaw-origin.yaw))<=.03000001;
    return LoopScore{1,1};
  });
  check(bounded,"rotated bounds exceeded during refinement");
  o=LoopSearchOptions{};calls=0;
  (void)search_loop_modes({0,0,0},o,[&](const auto& p){++calls;return twin(p);});
  check(calls<=13*13*11+8*7*26,"search budget exceeded");
  std::cout<<"Default search score evaluations: "<<calls<<'\n';

  // Actual point-cloud scoring against a synthetic, smooth cached wall field.
  constexpr int w=240;constexpr double res=.05,offset=-6;
  std::vector<float> field_distance(w*w);std::vector<double> field_score(w*w);
  std::vector<std::pair<double,double>> world,scan;
  for(int y=0;y<w;++y) for(int x=0;x<w;++x) {
    const double px=offset+(x+.5)*res,py=offset+(y+.5)*res;
    const double d=std::min(std::abs(px-3.025),std::abs(py-3.525));
    field_distance[y*w+x]=static_cast<float>(d);field_score[y*w+x]=std::exp(-.5*d*d/.04);
  }
  for(int k=40;k<180;k+=2) {world.emplace_back(3.025,offset+(k+.5)*res);world.emplace_back(offset+(k+.5)*res,3.525);}
  LoopFieldView field(field_distance,field_score,w,w,res,offset,offset);
  double max_error=0;const auto start=std::chrono::steady_clock::now();
  for(int trial=0;trial<12;++trial) {
    const PoseSample2 actual{.025+trial*.047,-.13+trial*.019,-.043+trial*.011};
    scan.clear();const double ca=std::cos(actual.yaw),sa=std::sin(actual.yaw);
    for(const auto& p:world) {const double x=p.first-actual.x,y=p.second-actual.y;scan.emplace_back(ca*x+sa*y,-sa*x+ca*y);}
    const auto matches=search_loop_modes({0,0,0},o,[&](const auto& p){return score_loop_scan(field,scan,p);});
    check(!matches.empty(),"wall scan rejected");
    const double error=std::hypot(matches[0].pose.x-actual.x,matches[0].pose.y-actual.y);
    max_error=std::max(max_error,error);
    check(error<.02 && std::abs(wrap_angle(matches[0].pose.yaw-actual.yaw))<.004,"subcell wall alignment");
  }
  const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  std::cout<<"12 synthetic wall cases: max translation error="<<max_error<<" elapsed_ms="<<ms<<'\n';
  check(score_loop_scan(field,std::vector<std::pair<double,double>>{},truth).score==0,"empty cloud");
  LoopQueryLedger ledger;
  double odds=1;
  for(int i=0;i<1000;++i) if(ledger.consume(42)) odds*=4;
  check(odds==4 && ledger.size()==1,"repeat event amplified evidence");
  check(ledger.contains(42) && !ledger.contains(43),"query association");
  check(ledger.consume(43) && ledger.size()==2,"new event suppressed");
  std::cout<<"PASS: "<<checks<<" loop search/evidence checks\n";
}
