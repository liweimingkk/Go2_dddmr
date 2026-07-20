#ifndef LEGO_LOAM_BOR_AXIS_SPLIT_FACTOR_H
#define LEGO_LOAM_BOR_AXIS_SPLIT_FACTOR_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace lego_loam_bor
{

// A relative pose measurement whose axes deliberately come from two sources:
// external odometry/IMU supplies rotation and world-frame X/Y, while scan or
// ground matching supplies world-frame Z.  Using a custom world-translation
// residual avoids the local-tangent-axis coupling of a masked Pose3 factor.
struct AxisSplitMeasurement
{
  gtsam::Rot3 relative_rotation;
  gtsam::Vector3 world_translation_delta = gtsam::Vector3::Zero();

  bool isFinite() const
  {
    return relative_rotation.matrix().allFinite() &&
      world_translation_delta.allFinite();
  }
};

struct AxisSplitScalarUpdate
{
  bool observable = false;
  bool clamped = false;
  double information = 0.0;
  double delta = 0.0;
};

inline AxisSplitScalarUpdate solveAxisSplitScalarUpdate(
  const double information,
  const double right_hand_side,
  const double minimum_information,
  const double maximum_absolute_step)
{
  AxisSplitScalarUpdate update;
  update.information = information;
  if (!std::isfinite(information) || !std::isfinite(right_hand_side) ||
    !std::isfinite(minimum_information) ||
    !std::isfinite(maximum_absolute_step) || minimum_information < 0.0 ||
    maximum_absolute_step <= 0.0 || information < minimum_information ||
    information <= 0.0)
  {
    return update;
  }

  const double unconstrained_delta = right_hand_side / information;
  if (!std::isfinite(unconstrained_delta)) {
    return update;
  }
  update.observable = true;
  update.delta = std::clamp(
    unconstrained_delta, -maximum_absolute_step, maximum_absolute_step);
  update.clamped = update.delta != unconstrained_delta;
  return update;
}

inline bool isFinitePose(const gtsam::Pose3 & pose)
{
  return pose.rotation().matrix().allFinite() &&
    pose.translation().allFinite();
}

inline AxisSplitMeasurement makeAxisSplitMeasurement(
  const gtsam::Pose3 & previous_odometry_pose,
  const gtsam::Pose3 & current_odometry_pose,
  const gtsam::Pose3 & previous_scan_pose,
  const gtsam::Pose3 & current_scan_pose)
{
  AxisSplitMeasurement measurement;
  measurement.relative_rotation = previous_odometry_pose.rotation().between(
    current_odometry_pose.rotation());
  measurement.world_translation_delta <<
    current_odometry_pose.x() - previous_odometry_pose.x(),
    current_odometry_pose.y() - previous_odometry_pose.y(),
    current_scan_pose.z() - previous_scan_pose.z();
  return measurement;
}

class AxisSplitFactor final :
  public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  AxisSplitFactor() = default;

  AxisSplitFactor(
    const gtsam::Key previous_key,
    const gtsam::Key current_key,
    const AxisSplitMeasurement & measurement,
    const gtsam::SharedNoiseModel & noise_model)
  : Base(noise_model, previous_key, current_key), measurement_(measurement)
  {
    if (!measurement_.isFinite()) {
      throw std::invalid_argument("axis-split measurement must be finite");
    }
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new AxisSplitFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & previous_pose,
    const gtsam::Pose3 & current_pose,
    boost::optional<gtsam::Matrix &> previous_jacobian = boost::none,
    boost::optional<gtsam::Matrix &> current_jacobian = boost::none) const override
  {
    gtsam::Matrix36 previous_rotation_jacobian;
    gtsam::Matrix36 current_rotation_jacobian;
    const gtsam::Rot3 & previous_rotation = previous_pose.rotation(
      previous_rotation_jacobian);
    const gtsam::Rot3 & current_rotation = current_pose.rotation(
      current_rotation_jacobian);

    gtsam::Matrix3 predicted_previous_jacobian;
    gtsam::Matrix3 predicted_current_jacobian;
    const gtsam::Rot3 predicted_relative_rotation = previous_rotation.between(
      current_rotation,
      predicted_previous_jacobian,
      predicted_current_jacobian);
    gtsam::Matrix3 rotation_error_jacobian;
    const gtsam::Vector3 rotation_error =
      measurement_.relative_rotation.localCoordinates(
      predicted_relative_rotation, boost::none, rotation_error_jacobian);

    gtsam::Matrix36 previous_translation_jacobian;
    gtsam::Matrix36 current_translation_jacobian;
    const gtsam::Point3 & previous_translation = previous_pose.translation(
      previous_translation_jacobian);
    const gtsam::Point3 & current_translation = current_pose.translation(
      current_translation_jacobian);
    const gtsam::Vector3 translation_error =
      (current_translation - previous_translation) -
      measurement_.world_translation_delta;

    if (previous_jacobian) {
      previous_jacobian->setZero(6, 6);
      previous_jacobian->topRows<3>() = rotation_error_jacobian *
        predicted_previous_jacobian * previous_rotation_jacobian;
      previous_jacobian->bottomRows<3>() = -previous_translation_jacobian;
    }
    if (current_jacobian) {
      current_jacobian->setZero(6, 6);
      current_jacobian->topRows<3>() = rotation_error_jacobian *
        predicted_current_jacobian * current_rotation_jacobian;
      current_jacobian->bottomRows<3>() = current_translation_jacobian;
    }

    gtsam::Vector6 error;
    error << rotation_error, translation_error;
    return error;
  }

  const AxisSplitMeasurement & measurement() const
  {
    return measurement_;
  }

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

private:
  AxisSplitMeasurement measurement_;
};

}  // namespace lego_loam_bor

#endif  // LEGO_LOAM_BOR_AXIS_SPLIT_FACTOR_H
