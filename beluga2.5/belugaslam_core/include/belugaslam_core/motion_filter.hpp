#ifndef BELUGASLAM_CORE_MOTION_FILTER_HPP
#define BELUGASLAM_CORE_MOTION_FILTER_HPP

// The caller computes displacement from the LAST INSERTED pose, after matching.
// Equality is still "similar", as in Cartographer's MotionFilter::IsSimilar.
// A backwards timestamp starts a new insertion interval instead of stalling it.
inline bool motion_filter_accepts(
    double elapsed_seconds, double translation, double rotation,
    double max_time_seconds, double max_translation, double max_rotation) {
  return elapsed_seconds < 0.0 || elapsed_seconds > max_time_seconds ||
         translation > max_translation || rotation > max_rotation;
}

#endif  // BELUGASLAM_CORE_MOTION_FILTER_HPP
