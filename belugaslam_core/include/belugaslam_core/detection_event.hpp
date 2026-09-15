#ifndef BELUGASLAM_CORE_DETECTION_EVENT_HPP
#define BELUGASLAM_CORE_DETECTION_EVENT_HPP
#include <cstddef>
#include <cstdint>

#include <sophus/se2.hpp>

namespace belugaslam {

enum class EventType {
  kSpatialCluster,  ///< A persistent spatial mode forked a new hypothesis.
  kLoopClosure      ///< A verified candidate installed a loop branch.
};

/** One thing the detectors did, recorded where and when it happened.
 *
 * The history is append-only: an event records that the detector produced this
 * association or this fork, not that the estimate still believes it. A loop branch
 * that deferred validation rejects thirty scans later keeps its event, which is
 * what makes the record usable for evaluating the detector separately from the
 * filter that consumes it.
 *
 * The timestamp is the scan stamp in nanoseconds rather than an rclcpp::Time so
 * that the core stays independent of ROS; the node converts on publication.
 */
struct DetectionEvent {
  std::size_t id = 0;
  EventType type = EventType::kSpatialCluster;
  Sophus::SE2d pose;
  std::int64_t timestamp_ns = 0;
  std::size_t hypothesis_id = 0;
};

}  // namespace belugaslam
#endif
