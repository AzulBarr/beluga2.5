#include "belugaslam_core/robust_tracking.hpp"
#include "belugaslam_core/derived_cache.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace belugaslam;
namespace {
int checks=0;
void check(bool condition,const char* message) {++checks;if(!condition) throw std::runtime_error(message);}
struct CacheData { std::vector<double> values; std::size_t bytes() const {return values.capacity()*sizeof(double);} };
}
int main() {
  constexpr int size=240; constexpr double resolution=.05,origin=-6;
  std::vector<float> cells(size*size);
  ScanPoints endpoints;
  for (int i=20;i<180;++i) {
    cells[i*size+180]=5; cells[190*size+i]=5;
    endpoints.emplace_back(origin+180.5*resolution,origin+(i+.5)*resolution);
    endpoints.emplace_back(origin+(i+.5)*resolution,origin+190.5*resolution);
  }
  TrackingField field(cells,size,size,resolution,origin,origin);
  TrackingOptions options; RecoveryOptions recovery;
  PoseSample2 truth{.78,-.24,.10}; ScanPoints scan;
  for (const auto& [x,y]:endpoints) {
    const double dx=x-truth.x,dy=y-truth.y,c=std::cos(truth.yaw),s=std::sin(truth.yaw);
    scan.emplace_back(c*dx+s*dy,-s*dx+c*dy);
  }
  const auto normal=match_tracking_scan(field,scan,{0,0,0},options);
  const auto start=std::chrono::steady_clock::now();
  const auto recovered=recover_tracking_scan(field,scan,{0,0,0},options,recovery);
  const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  std::cout<<"Normal pose "<<normal.pose.x<<' '<<normal.pose.y<<" overlap "<<normal.score.overlap
           <<"; recovery "<<recovered.pose.x<<' '<<recovered.pose.y<<' '<<recovered.pose.yaw
           <<" accepted "<<recovered.accepted<<" in "<<ms<<" ms\n";
  check(std::hypot(normal.pose.x-truth.x,normal.pose.y-truth.y)>.25,"fixture must exceed ordinary search support");
  check(recovered.accepted,"bounded recovery rejected observable displaced scan");
  check(std::hypot(recovered.pose.x-truth.x,recovered.pose.y-truth.y)<.12,"recovery translation error");
  check(std::abs(wrap_angle(recovered.pose.yaw-truth.yaw))<.02,"recovery rotation error");
  check(recovered.score.overlap>=recovery.min_overlap,"recovery lacks overlap");
  ScanPoints unsupported;for(int i=0;i<40;++i) unsupported.emplace_back(100+i,100);
  check(!recover_tracking_scan(field,unsupported,{0,0,0},options,recovery).accepted,"unsupported recovery accepted");

  std::vector<float> repeated(size*size);ScanPoints repeated_scan;
  for (int i=0;i<9;++i) {
    repeated_scan.emplace_back(.025,.025+i*resolution);
    repeated_scan.emplace_back(.025+i*resolution,.025);
    for (int center:{107,133}) {repeated[(120+i)*size+center]=5;repeated[120*size+center+i]=5;}
  }
  TrackingField ambiguous(repeated,size,size,resolution,origin,origin);
  const auto ambiguous_result=recover_tracking_scan(ambiguous,repeated_scan,{0,0,0},options,recovery);
  check(!ambiguous_result.accepted,"equally supported separated recovery modes must be rejected");

  // A shared frozen cache builds once even when several search workers race.
  DerivedCache<CacheData> cache;std::atomic<int> builds{0};
  auto builder=[&] {++builds;return CacheData{std::vector<double>(1000,.125)};};
  std::vector<std::thread> workers;
  for(int i=0;i<8;++i) workers.emplace_back([&] {for(int j=0;j<20;++j) (void)cache.acquire(builder);});
  for(auto& worker:workers) worker.join();
  check(builds==1,"cache built more than once under concurrent acquisition");
  auto retained=cache.acquire(builder);const auto first_use=cache.statistics().second;
  check(cache.statistics().first>=8000,"cache byte accounting");
  cache.release();check(cache.statistics().first==0,"eviction did not release cache ownership");
  check(retained->values[999]==.125,"eviction invalidated a live reader");
  auto rebuilt=cache.acquire(builder);
  check(builds==2 && rebuilt->values==retained->values,"eviction/rebuild changed payload");
  check(cache.statistics().second>first_use,"cache recency did not advance");
  for(int i=0;i<1000;++i) {cache.release();(void)cache.acquire(builder);}
  check(cache.statistics().first==rebuilt->bytes(),"repeated eviction accumulated retained payloads");
  std::cout<<"PASS: "<<checks<<" late-run recovery/cache checks\n";
}
