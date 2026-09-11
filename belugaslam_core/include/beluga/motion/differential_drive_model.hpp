// Copyright 2022-2023 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BELUGA_MOTION_DIFFERENTIAL_DRIVE_MODEL_HPP
#define BELUGA_MOTION_DIFFERENTIAL_DRIVE_MODEL_HPP

#include <beluga/motion/differential_drive_distribution.hpp>
#include <random>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <sophus/se3.hpp>
#include <tuple>

#include <beluga/type_traits/tuple_traits.hpp>

#include <beluga/3d_embedding.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <type_traits>

/**
 * \file
 * \brief Implementation of a differential drive odometry motion model.
 */

namespace beluga {

/// Sampled odometry model for a differential drive.
/**
 * Supports 2D and (flattened) 3D state types.
 * This class satisfies \ref MotionModelPage.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 5.4.2.
 *
 * \tparam StateType Type for particle's state. Either Sophus::SE2d or Sophus::SE3d.
 */
template <class StateType = Sophus::SE2d>
class DifferentialDriveModel {
  static_assert(
      std::is_same_v<StateType, Sophus::SE2d> or std::is_same_v<StateType, Sophus::SE3d>,
      "Differential model only supports SE2 and SE3 state types.");

 public:
  /// 2D or flattened 3D pose as motion model state (to match that of the particles).
  using state_type = StateType;

  /// Current and previous odometry estimates as motion model control action.
  using control_type = std::tuple<state_type, state_type>;

  /// Parameter type that the constructor uses to configure the motion model.
  using param_type = DifferentialDriveModelParam;

  /// Constructs a DifferentialDriveModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::DifferentialDriveModelParam for details.
   */
  explicit DifferentialDriveModel(const param_type& params) : params_{params} {
    for (double value : {params.rotation_noise_from_rotation, params.rotation_noise_from_translation,
                         params.translation_noise_from_translation, params.translation_noise_from_rotation,
                         params.distance_threshold}) {
      if (!std::isfinite(value) || value < 0) throw std::invalid_argument("Invalid differential-drive noise/threshold");
    }
  }

  /// Computes a state sampling function conditioned on a given control action.
  /**
   * \tparam Control A tuple-like container matching the model's `control_type`.
   * \param action Control action to condition the motion model with.
   * \return a callable satisfying \ref StateSamplingFunctionPage.
   */
  template <class Control, typename = common_tuple_type_t<Control, control_type>>
  [[nodiscard]] auto operator()(const Control& action) const {
    const auto& [pose, previous_pose] = action;
    if constexpr (std::is_same_v<state_type, Sophus::SE2d>) {
      return sampling_fn_2d(pose, previous_pose);
    } else {
      return sampling_fn_3d(pose, previous_pose);
    }
  }

  [[nodiscard]] DifferentialDriveDistribution2d increment_distribution(const control_type& action) const {
    if constexpr (std::is_same_v<state_type, Sophus::SE2d>)
      return distribution_2d(std::get<0>(action), std::get<1>(action));
    else
      return distribution_2d(To2d(std::get<0>(action)), To2d(std::get<1>(action)));
  }

 private:
  using control_type_2d = std::tuple<Sophus::SE2d, Sophus::SE2d>;
  using control_type_3d = std::tuple<Sophus::SE3d, Sophus::SE3d>;

  [[nodiscard]] auto sampling_fn_3d(const Sophus::SE3d& pose, const Sophus::SE3d& previous_pose) const {
    const auto current_pose_2d = To2d(pose);
    const auto previous_pose_pose_2d = To2d(previous_pose);
    const auto two_d_sampling_fn = sampling_fn_2d(current_pose_2d, previous_pose_pose_2d);
    return [=](const state_type& state, auto& gen) { return To3d(two_d_sampling_fn(To2d(state), gen)); };
  }

  [[nodiscard]] DifferentialDriveDistribution2d distribution_2d(
      const Sophus::SE2d& pose, const Sophus::SE2d& previous_pose) const {
    const auto translation = pose.translation() - previous_pose.translation();
    const double distance = translation.norm();
    const auto first = distance > 0.0 ?
        Sophus::SO2d{std::atan2(translation.y(), translation.x())} * previous_pose.so2().inverse() : Sophus::SO2d{};
    const auto turn = pose.so2() * previous_pose.so2().inverse();
    const auto second=turn*first.inverse();
    const auto first_noise=distance>params_.distance_threshold ? first : Sophus::SO2d{};
    const auto second_noise=distance>params_.distance_threshold ? second : turn;
    // Keep the original Sophus variance calculation and RNG sampling path;
    // importance evaluation consumes these exact same parameters.
    const auto variance=[](const Sophus::SO2d& rotation) {
      const auto flipped=rotation*Sophus::SO2d{Sophus::Constants<double>::pi()};
      const double v=std::min(std::abs(rotation.log()),std::abs(flipped.log()));
      return v*v;
    };
    const double d2=distance*distance;
    return {first.log(),distance,second.log(),
      std::sqrt(params_.rotation_noise_from_rotation*variance(first_noise)+
                params_.rotation_noise_from_translation*d2),
      std::sqrt(params_.translation_noise_from_translation*d2+
                params_.translation_noise_from_rotation*(variance(first_noise)+variance(second_noise))),
      std::sqrt(params_.rotation_noise_from_rotation*variance(second_noise)+
                params_.rotation_noise_from_translation*d2)};
  }

  [[nodiscard]] auto sampling_fn_2d(const Sophus::SE2d& pose, const Sophus::SE2d& previous_pose) const {
    const auto distribution = distribution_2d(pose, previous_pose);
    using DistributionParam = std::pair<double, double>;
    const DistributionParam first_rotation_params{distribution.first_mean, distribution.first_sigma};
    const DistributionParam translation_params{distribution.distance_mean, distribution.distance_sigma};
    const DistributionParam second_rotation_params{distribution.second_mean, distribution.second_sigma};

    return [=](const auto& state, auto& gen) {
      // Zero noise is deterministic. A distribution cache must not leak draws
      // between independent filters that happen to share the calling thread.
      const auto draw = [&](const DistributionParam& p) {
        if (p.second == 0.0) return p.first;
        return std::normal_distribution<double>{p.first, p.second}(gen);
      };
      const auto first_rotation = Sophus::SO2d{draw(first_rotation_params)};
      const auto translation = Eigen::Vector2d{draw(translation_params), 0.0};
      const auto second_rotation = Sophus::SO2d{draw(second_rotation_params)};
      return state * Sophus::SE2d{first_rotation, Eigen::Vector2d{0.0, 0.0}} *
             Sophus::SE2d{second_rotation, translation};
    };
  }
  param_type params_;

};

/// Alias for a 2D differential drive model, for convinience.
using DifferentialDriveModel2d = DifferentialDriveModel<Sophus::SE2d>;

/// Alias for a 3D differential drive model, for convinience.
using DifferentialDriveModel3d = DifferentialDriveModel<Sophus::SE3d>;

}  // namespace beluga

#endif
