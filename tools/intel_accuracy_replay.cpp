// Deterministic delivery of every Intel scan to the production BelugaSLAM core.
// Input: acquisition_ns odom_x odom_y odom_yaw count ranges... (one scan/line).
// Reference trajectories are deliberately not an input to this executable.
#include "belugaslam_core/fastslam_oc_grid_core.hpp"
#include <filesystem>
#include <sstream>

namespace fs=std::filesystem;
int main(int argc,char** argv) {
  try {
    if(argc<11 || argc>19) throw std::invalid_argument(
      "Usage: intel_accuracy_replay input output particles hypotheses seed loops frontend_mode effective_beams submap_scans prior_information_scale [bayes|heuristic [distance|probability_ceres [occupied_space_weight [voxel_size [fixed|odometry [odom_translation_sigma [odom_rotation_sigma [scan_proposal:on|off]]]]]]]]");
    const fs::path input=argv[1], output=argv[2];
    if(!fs::is_directory(output)) throw std::invalid_argument("Output directory must exist");
    FastSLAMParams params;
    params.min_particles=5;params.max_particles=std::stoul(argv[3]);
    params.max_hypotheses=std::stoul(argv[4]);params.random_seed=std::stoul(argv[5]);
    const std::string loops=argv[6];
    if(loops!="off" && loops!="belief" && loops!="map" && loops!="geometry")
      throw std::invalid_argument("Invalid loop mode");
    params.enable_loop_closure=loops!="off";params.enable_pgo=loops!="off";
    params.loop_verifier_mode=loops=="off" ? "belief" : loops;
    params.frontend_pose_mode=argv[7];params.tracking.effective_beams=std::stod(argv[8]);
    params.submap_num_range_data=std::stoi(argv[9]);
    params.tracking.prior_information_scale=std::stod(argv[10]);
    if(argc>=12) params.loop_update_mode=argv[11];
    if(argc>=13) params.tracking_matcher=argv[12];
    if(argc>=14) params.probability_matching.occupied_space_weight=std::stod(argv[13]);
    if(argc>=15) params.probability_matching.voxel_size=std::stod(argv[14]);
    if(argc>=16) params.tracking_prior_mode=argv[15];
    if(argc>=17) params.odometry_prior.translation_sigma=std::stod(argv[16]);
    if(argc>=18) params.odometry_prior.rotation_sigma=std::stod(argv[17]);
    if(argc>=19) {
      const std::string mode=argv[18];
      if(mode!="on" && mode!="off") throw std::invalid_argument("scan_proposal must be on or off");
      params.scan_informed_proposal=mode=="on";
    }
    // Keep these coefficients identical to the PF MotionModel constructed below.
    params.odometry_prior.alpha1=.1;params.odometry_prior.alpha2=.05;
    params.odometry_prior.alpha3=.1;params.odometry_prior.alpha4=.05;
    params.odometry_prior.rotation_distance_threshold=.01;
    params.loop_bayes_diagnostics_path=(output/"bayes.csv").string();
    params.output_selection_mode="pose_risk";params.worker_threads=2;
    params.loop_diagnostics_path=(output/"loops.csv").string();
    params.tracking_diagnostics_path=(output/"tracking.csv").string();
    BelugaSLAM slam{BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{.1,.05,.1,.05,.01}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},params};
    std::ifstream source(input);
    if(!source) throw std::runtime_error("Cannot open input");
    std::ofstream perf(output/"performance.csv"), tum(output/"optimized_trajectory.tum");
    if(!perf||!tum) throw std::runtime_error("Cannot open output");
    perf<<std::setprecision(17)<<"stamp_ns,status,output_x,output_y,output_yaw,selected_hypothesis,particles,hypotheses,total_ms,tracking_status,backend_ms,candidates,trials\n";
    std::int64_t previous_stamp=0; bool first=true;
    Sophus::SE2d previous_odom;
    std::size_t count=0;std::string line;
    while(std::getline(source,line)) {
      std::istringstream row(line);
      std::int64_t stamp;double x,y,yaw;std::size_t n;
      if(!(row>>stamp>>x>>y>>yaw>>n) || n<2 || n>100000 ||
          !std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw))
        throw std::runtime_error("Invalid input at scan "+std::to_string(count));
      if(!first && stamp<=previous_stamp) throw std::runtime_error("Non-increasing input timestamps");
      const auto begin=std::chrono::steady_clock::now();
      const Sophus::SE2d odom{Sophus::SO2d{yaw},Eigen::Vector2d{x,y}};
      // Match LaserScan float32 range/angle conversion in the supplied Intel ROS launch.
      const float angle_min=-Sophus::Constants<double>::pi()/2;
      const float angle_increment=Sophus::Constants<double>::pi()/180;
      BelugaSLAM::measurement_type scan;scan.reserve(n);
      for(std::size_t i=0;i<n;++i) {
        std::string token;if(!(row>>token)) throw std::runtime_error("Missing laser range");
        const float range=static_cast<float>(std::stod(token));
        if(std::isfinite(range) && range>0.1 && range<30.0) {
          const double angle=static_cast<double>(angle_min)+i*static_cast<double>(angle_increment);
          scan.emplace_back(range*std::cos(angle),range*std::sin(angle));
        }
      }
      std::string extra;if(row>>extra) throw std::runtime_error("Extra input fields");
      if(scan.empty()) throw std::runtime_error("Empty usable scan; no scan may be silently omitted");
      if(first) previous_odom=odom;
      slam.sample_motion_model(std::make_tuple(odom,previous_odom));
      slam.measurement_model_map(scan);
      const auto events=slam.update_occupancy_grid(scan,static_cast<double>(stamp)*1e-9,stamp);
      const auto backend_start=std::chrono::steady_clock::now();
      slam.post_update(scan,events);
      const double backend_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-backend_start).count();
      slam.resample();
      const auto pose=slam.best_pose();
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      perf<<stamp<<",processed,"<<pose.translation().x()<<','<<pose.translation().y()<<','<<pose.so2().log()<<','
          <<slam.best_hypothesis_id()<<','<<slam.particles().size()<<','<<slam.get_active_hypotheses_count()<<','
          <<ms<<','<<slam.best_tracking_status()<<','<<backend_ms<<','<<slam.backend_timing().candidates<<','
          <<slam.backend_timing().trials<<'\n';
      previous_odom=odom;previous_stamp=stamp;first=false;++count;
      if(count%100==0) {
        perf.flush();std::cout<<"Processed "<<count<<" scans; last="<<ms<<" ms; hypotheses="
          <<slam.get_active_hypotheses_count()<<std::endl;
      }
    }
    if(!count || !source.eof()) throw std::runtime_error("Empty input or input read failure");
    const bool finalized=slam.finalize_trajectory();
    const auto written=slam.write_optimized_trajectory(tum);
    perf.flush();tum.flush();
    if(!perf||!tum||written!=count) throw std::runtime_error("Incomplete trajectory export");
    const auto& grid=slam.best_occupancy_grid();
    std::ofstream pgm(output/"map.pgm",std::ios::binary), yaml(output/"map.yaml");
    pgm<<"P5\n"<<grid.width()<<' '<<grid.height()<<"\n255\n";
    for(int row=grid.height()-1;row>=0;--row) for(int col=0;col<grid.width();++col) {
      const auto value=grid.data()[static_cast<std::size_t>(row)*grid.width()+col];
      const unsigned char color=value<0 ? 205 : value>=65 ? 0 : 254;
      pgm.write(reinterpret_cast<const char*>(&color),1);
    }
    yaml<<std::setprecision(17)<<"image: map.pgm\nresolution: "<<grid.resolution()<<"\norigin: ["
      <<grid.origin().translation().x()<<", "<<grid.origin().translation().y()<<", "<<grid.origin().so2().log()
      <<"]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    if(!pgm||!yaml) throw std::runtime_error("Map export failed");
    std::cout<<"REPLAY_COMPLETE scans="<<count<<" trajectory="<<written<<" final_pgo="
      <<(finalized ? "true" : "false")<<std::endl;
    return finalized ? 0 : 2;
  } catch(const std::exception& e) {
    std::cerr<<"REPLAY_ERROR: "<<e.what()<<std::endl;return 1;
  }
}
