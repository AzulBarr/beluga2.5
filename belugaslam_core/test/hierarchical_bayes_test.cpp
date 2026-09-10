#include "belugaslam_core/hierarchical_bayes.hpp"
#include "belugaslam_core/particle_proposal.hpp"
#include "belugaslam_core/loop_belief.hpp"
#include "belugaslam_core/validation_map.hpp"
#include <iostream>
#include <random>
#include <string>

namespace {
int checks=0;
void require(bool condition,const std::string& message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
bool near(double a,double b,double tolerance=1e-12) { return std::abs(a-b)<tolerance; }
template<class F> void rejects(F function) {
  bool threw=false;try { function(); } catch (const std::invalid_argument&) {threw=true;}
  require(threw,"invalid input must fail explicitly");
}
}
int main() {
  using namespace belugaslam;
  try {
    const auto update=update_conditional({std::log(.25),std::log(.75)}, {std::log(.8),std::log(.2)});
    require(near(std::exp(update.log_evidence),.35),"predictive evidence includes prior conditional weights");
    require(near(std::exp(update.log_weights[0]),4./7.),"conditional posterior uses predictive normalizer");
    std::vector<double> graph_logs{std::log(.4)+update.log_evidence,std::log(.6)+std::log(.1)};
    normalize_log_weights(graph_logs);
    require(near(std::exp(graph_logs[0]),.7),"graph prior is applied exactly once");

    // The same distribution represented with different particle quotas.
    const auto duplicated=update_conditional(
      {std::log(.125),std::log(.125),std::log(.375),std::log(.375)},
      {std::log(.8),std::log(.8),std::log(.2),std::log(.2)});
    require(near(update.log_evidence,duplicated.log_evidence),"particle duplication cannot change graph evidence");
    const auto unequal=update_conditional({std::log(4.),std::log(12.)},{std::log(.8),std::log(.2)});
    require(near(update.log_evidence,unequal.log_evidence),"conditional prior is normalized before marginalization");

    const auto extreme=update_conditional({-1000.,0.},{-1000.,-3000.});
    require(std::isfinite(extreme.log_evidence),"extreme evidence stays in log space");
    require(extreme.log_weights[0]>-1e-10 && near(extreme.log_weights[1],-1000.),"underflowed conditional support remains represented in logs");
    std::vector<double> tiny{0.,-2000.};normalize_log_weights(tiny);
    require(tiny[1]==-2000.,"tiny graph log mass is preserved");
    const auto neutral=update_conditional({std::log(.1),std::log(.9)},{0.,0.});
    require(near(neutral.log_evidence,0.) && near(std::exp(neutral.log_weights[0]),.1),"missing evidence is neutral at both levels");
    rejects([] {update_conditional({0.},{});});
    rejects([] {std::vector<double> v{-INFINITY,-INFINITY};normalize_log_weights(v);});
    rejects([] {log_sum_exp({NAN,0.});});

    std::mt19937 rng(42);
    const auto proposal=select_motion_proposal({std::log(.1),std::log(.9)},rng);
    require(near(std::exp(proposal.log_evidence),.5),"motion proposal evidence is mean, not selected or maximum likelihood");
    auto quotas=allocate_particle_quotas({.999,.001},30);
    require(quotas[0]+quotas[1]==30 && quotas[1]>=1,"minor mode receives protected integer quota");
    const auto indices=systematic_indices({.2,.8},300,rng);
    require(std::count(indices.begin(),indices.end(),0)==60,"conditional systematic resampling respects its input posterior");

    SequentialLoopOptions options;options.validate();
    require(decide_loop(.999,9,9,options)==LoopDecision::pending,"confidence cannot bypass minimum future scans");
    require(decide_loop(.96,10,10,options)==LoopDecision::accepted,"accept posterior threshold after enough evidence");
    require(decide_loop(.04,10,10,options)==LoopDecision::rejected,"reject posterior threshold after enough evidence");
    require(decide_loop(.5,30,30,options)==LoopDecision::undecided,"bounded ambiguous window reports undecided");
    require(decide_loop(.99,1,30,options)==LoopDecision::undecided,"missing coverage cannot masquerade as 30 evidence scans");
    require(decide_loop(.05,10,10,options)==LoopDecision::rejected,"rejection boundary is inclusive");
    require(decide_loop(.95,10,10,options)==LoopDecision::accepted,"acceptance boundary is inclusive");
    rejects([] {SequentialLoopOptions o;o.beta=0;o.validate();});
    rejects([] {SequentialLoopOptions o;o.min_scans=31;o.validate();});

    constexpr int width=100,height=80;
    std::vector<float> cells(width*height,-1.f);
    for(int y=0;y<height;++y) cells[y*width+70]=5;
    ValidationMap frozen(cells,width,height,.1,-5.,-4.,{0,0,0});
    ValidationMap rotated(cells,width,height,.1,-5.,-4.,{3,-2,.7});
    TrackingOptions tracking;
    ScanPoints scan;
    for(int y=25;y<55;++y) scan.emplace_back(2.05,-4.+(y+.5)*.1);
    const double correct=frozen.log_likelihood(scan,{0,0,0},tracking,.1);
    const double wrong=frozen.log_likelihood(scan,{.6,0,0},tracking,.1);
    require(correct>wrong,"historical reference discriminates displaced loop predictions");
    require(near(correct,rotated.log_likelihood(scan,{3,-2,.7},tracking,.1)),"frozen reference likelihood is invariant to a common world-frame transform");
    std::fill(cells.begin(),cells.end(),0.f);
    require(near(correct,frozen.log_likelihood(scan,{0,0,0},tracking,.1)),"later tracking-map writes cannot change the frozen validation field");
    require(frozen.observed_endpoint({0,0,0},scan.front()),"observed historical endpoint is eligible");
    require(!frozen.observed_endpoint({0,0,0},{100.,100.}),"outside snapshot is missing evidence");
    ValidationMap empty(cells,width,height,.1,-5.,-4.,{0,0,0});
    require(!empty.usable() && !empty.observed_endpoint({0,0,0},scan.front()),"unknown reference cannot create confidence");
    require(near(wrong*.5,frozen.log_likelihood(scan,{.6,0,0},tracking,.05)),"beta is applied once");

    // End-to-end numerical decision with unequal particle budgets and repeated
    // conditional resampling. This intentionally uses the production kernel.
    for(bool loop_is_correct : {false,true}) {
      std::vector<double> mass{std::log(.5),std::log(.5)};
      LoopDecision decision=LoopDecision::pending;
      for(std::size_t t=1;t<=10;++t) {
        for(std::size_t h=0;h<2;++h) {
          const std::size_t count=h==0?29:1;
          const double likelihood=(h==1)==loop_is_correct ? correct : wrong;
          const auto result=update_conditional(std::vector<double>(count,-std::log(double(count))),
                                               std::vector<double>(count,likelihood));
          mass[h]+=result.log_evidence;
        }
        normalize_log_weights(mass);
        decision=decide_loop(std::exp(mass[1]),t,t,options);
        if(t<10) require(decision==LoopDecision::pending,"resampling or extreme odds cannot decide early");
      }
      require(decision==(loop_is_correct?LoopDecision::accepted:LoopDecision::rejected),"future evidence chooses the supported association, independently of quotas");
    }
    std::cout << "hierarchical_bayes_test: " << checks << " checks passed\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
