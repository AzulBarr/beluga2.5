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

/** Which side of its fork this event represents.
 *
 * A loop closure forks into two: the branch that accepts the association and the
 * branch that carries on without it. Both are real alternatives the filter has to
 * choose between, so both get an event, and either can be the one born weaker.
 */
enum class EventSide {
  kSpatialChild,  ///< the hypothesis a spatial mode forked off its parent
  kLoopChild,     ///< the branch that applied the loop correction
  kLoopNull       ///< the same parent carrying on without the loop
};

/** How the fork this event created turned out, against the side it forked from.
 *
 * "Won" means this hypothesis outlived its rival, or still outweighed it when the
 * run ended; "lost" means the opposite. Crossed with the mass it was born with,
 * the four values separate the forks that changed the estimate's mind from the
 * ones that merely confirmed it. kPending is a fork still undecided.
 */
enum class EventOutcome {
  kPending,
  kWeakerWon,     ///< born with less mass than its rival, outlived or outweighed it
  kWeakerLost,    ///< born with less mass, pruned or still behind at the end
  kStrongerWon,   ///< born with more mass and kept it
  kStrongerLost   ///< born with more mass and still lost
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
  EventSide side = EventSide::kSpatialChild;
  Sophus::SE2d pose;
  std::int64_t timestamp_ns = 0;
  std::size_t hypothesis_id = 0;

  /// The side of the fork this hypothesis was created against: its parent for a
  /// spatial fork, the no-loop branch of the same parent for a loop closure.
  /// The first fork of a run has hypothesis 0 as its rival, so the id alone cannot
  /// say whether the pair was recorded.
  bool has_rival = false;
  std::size_t rival_id = 0;
  double birth_mass = 0, rival_birth_mass = 0;
  bool born_weaker = false;
  EventOutcome outcome = EventOutcome::kPending;
};

}  // namespace belugaslam
#endif
