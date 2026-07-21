/*
 * Copyright (c) 2018, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>
#include <mcl_3dl/flat_ground.h>

namespace mcl_3dl
{
void LidarMeasurementModelLikelihood::loadConfig(
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter)
{

  logger_ = m_logger;
  parameter_ = m_parameter;

  parameter_->declare_parameter("likelihood.num_points_global", rclcpp::ParameterValue(0));
  rclcpp::Parameter num_points_global = parameter_->get_parameter("likelihood.num_points_global");
  num_points_global_ = num_points_global.as_int();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.num_points_global: %d", num_points_global_);

  num_points_default_ = num_points_;

  parameter_->declare_parameter("likelihood.threshold_for_trusted_ground", rclcpp::ParameterValue(0));
  rclcpp::Parameter threshold_for_trusted_ground = parameter_->get_parameter("likelihood.threshold_for_trusted_ground");
  threshold_for_trusted_ground_ = threshold_for_trusted_ground.as_int();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.threshold_for_trusted_ground: %d", threshold_for_trusted_ground_);

  parameter_->declare_parameter("likelihood.radius_of_ground_search", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter radius_of_ground_search = parameter_->get_parameter("likelihood.radius_of_ground_search");
  radius_of_ground_search_ = radius_of_ground_search.as_double();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.radius_of_ground_search: %.2f", radius_of_ground_search_);

  parameter_->declare_parameter("likelihood.match_dist_min", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter match_dist_min = parameter_->get_parameter("likelihood.match_dist_min");
  match_dist_min_ = match_dist_min.as_double();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.match_dist_min: %.2f", match_dist_min_);

  parameter_->declare_parameter("likelihood.match_dist_flat", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter match_dist_flat = parameter_->get_parameter("likelihood.match_dist_flat");
  match_dist_flat_ = match_dist_flat.as_double();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.match_dist_flat: %.2f", match_dist_flat_);

}
void LidarMeasurementModelLikelihood::setGlobalLocalizationStatus(
    const int num_particles,
    const int current_num_particles)
{
  if (current_num_particles <= num_particles)
  {
    num_points_ = num_points_default_;
    return;
  }
  int num = num_points_default_ * num_particles / current_num_particles;
  if (num < num_points_global_)
    num = num_points_global_;

  num_points_ = num;
}

LidarMeasurementResult LidarMeasurementModelLikelihood::measure(
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree,
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree_ground,
    pcl::PointCloud<pcl::Normal>& normals,
    std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr> pcl_segmentations,
    const State6DOF& s) const
{
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_flat_new_type(new pcl::PointCloud<mcl_3dl::pcl_t>);
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_sharp_new_type(new pcl::PointCloud<mcl_3dl::pcl_t>);

  float score_like = 0.0f;
  *pc_flat_new_type = (*pcl_segmentations[std::string("flat")]);
  s.transform(*pc_flat_new_type);

  *pc_less_sharp_new_type = (*pcl_segmentations[std::string("less_sharp")]);
  s.transform(*pc_less_sharp_new_type);

  // In 2.5D the ground reference is below base_link. Treating base_link itself
  // as the ground point is what previously pulled the estimate down by the
  // robot's clearance and allowed map normals to tilt roll/pitch.
  const double expected_ground_z = s.pos_.z_ -
      (flat_ground_mode_ ? base_link_height_ : 0.0);
  const FlatGroundEstimate ground = estimateFlatGround(
      kdtree_ground, normals, s.pos_.x_, s.pos_.y_, expected_ground_z,
      radius_of_ground_search_,
      static_cast<std::size_t>(std::max(1, threshold_for_trusted_ground_)));
  float pos_weight = 1.0f;
  if (ground.valid)
  {
    const double height_error = std::abs(expected_ground_z - ground.z);
    if (flat_ground_mode_)
    {
      pos_weight = static_cast<float>(std::max(0.01, 1.0 - std::min(1.0, height_error)));
    }
    else
    {
      const Vec3 pose_up = s.rot_.normalized() * Vec3(0.0, 0.0, 1.0);
      const double alignment = std::clamp(
          static_cast<double>(pose_up.dot(ground.normal)), -1.0, 1.0);
      const double normal_error = std::acos(alignment);
      pos_weight = static_cast<float>(std::max(
          0.01,
          (1.0 - std::min(1.0, height_error)) *
          (1.0 - std::min(1.0, normal_error))));
    }
  }
  else if (flat_ground_mode_)
  {
    pos_weight = 0.01f;
  }
  else
  {
    pcl_t particle_pose;
    particle_pose.x = s.pos_.x_;
    particle_pose.y = s.pos_.y_;
    particle_pose.z = s.pos_.z_;
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    if (kdtree.nearestKSearch(
          particle_pose, 1, nearest_index, nearest_squared_distance) > 0)
    {
      pos_weight = std::max(0.01f, 1.0f - std::sqrt(nearest_squared_distance[0]));
    }
    else
    {
      pos_weight = 0.01f;
    }
  }

  size_t num = 0;
  double residual_sum = 0.0;

  for (auto& p : pc_flat_new_type->points)
  {
    std::vector<int> id;
    std::vector<float> sqdist;
    const int found = ground.valid ?
        kdtree_ground.radiusSearch(p, match_dist_min_, id, sqdist, 1) :
        kdtree.radiusSearch(p, match_dist_min_, id, sqdist, 1);
    const float nearest_distance = found > 0 ?
        std::sqrt(sqdist.front()) : match_dist_min_;
    residual_sum += std::min(nearest_distance, match_dist_min_);
    if (found > 0)
    {
      const float dist = match_dist_min_ - std::max(nearest_distance, match_dist_flat_);
      if (dist < 0.0)
      {
        continue;
      }
      score_like += dist * dist;
      num++;
    }
  }

  for (auto& p : pc_less_sharp_new_type->points)
  {
    std::vector<int> id;
    std::vector<float> sqdist;
    const int found = kdtree.radiusSearch(p, match_dist_min_, id, sqdist, 1);
    const float nearest_distance = found > 0 ?
        std::sqrt(sqdist.front()) : match_dist_min_;
    residual_sum += std::min(nearest_distance, match_dist_min_);
    if (found > 0)
    {
      const float dist = match_dist_min_ - std::max(nearest_distance, match_dist_flat_);
      if (dist < 0.0)
      {
        continue;
      }
      const float point_weight = std::max(0.05f, std::abs(p.intensity));
      score_like += dist * dist / point_weight;
      num++;
    }
  }
  const std::size_t total_points =
      pc_flat_new_type->points.size() + pc_less_sharp_new_type->points.size();
  const float match_ratio = total_points > 0 ?
      static_cast<float>(num) / static_cast<float>(total_points) : 0.0f;
  const float residual = total_points > 0 ?
      static_cast<float>(residual_sum / static_cast<double>(total_points)) :
      std::numeric_limits<float>::infinity();

  return LidarMeasurementResult(
      score_like * pos_weight, match_ratio, residual, ground.valid,
      static_cast<float>(ground.z), ground.normal,
      static_cast<float>(ground.roughness));
}
}  // namespace mcl_3dl
