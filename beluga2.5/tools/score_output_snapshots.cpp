// Offline frontend-snapshot decisions using the same helper as the ROS core.
// Input: sequence count, followed by count lines of id mass x y, repeated to EOF.
// This does not replay tracking, PGO, resampling, maps, or the ROS output pipeline.
#include "belugaslam_core/output_selection.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>

int main() {
  try {
    std::cout << std::setprecision(17)
              << "sequence,map_hypothesis,risk_hypothesis,map_position_risk_m2,minimum_position_risk_m2\n";
    std::size_t sequence, count;
    while (std::cin >> sequence) {
      if (!(std::cin >> count) || count == 0 || count > 10000)
        throw std::invalid_argument("Invalid snapshot size");
      std::vector<belugaslam::OutputPoseHypothesis> h(count);
      for (auto& p : h) if (!(std::cin >> p.id >> p.mass >> p.x >> p.y))
        throw std::invalid_argument("Incomplete snapshot");
      std::sort(h.begin(),h.end(),[](const auto& a,const auto& b){return a.id<b.id;});
      for (std::size_t i=1;i<h.size();++i) if(h[i-1].id==h[i].id)
        throw std::invalid_argument("Duplicate hypothesis ID");
      const auto result=belugaslam::select_output_pose(h);
      std::cout << sequence << ',' << h[result.map_index].id << ',' << h[result.risk_index].id << ','
                << result.map_position_risk << ',' << result.minimum_position_risk << '\n';
    }
    if (!std::cin.eof()) throw std::invalid_argument("Invalid input");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';return 1;
  }
}
