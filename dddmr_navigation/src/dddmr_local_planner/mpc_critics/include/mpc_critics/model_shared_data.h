/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef MPC_CRITICS_MODEL_SHARED_DATA_H_
#define MPC_CRITICS_MODEL_SHARED_DATA_H_

#include "rclcpp/rclcpp.hpp"

/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

/*path for trajectory*/
#include <base_trajectory/trajectory.h>
/*For tf2::matrix3x3 as quaternion to euler*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*For robot state*/
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <pcl/point_cloud.h>

/*kdtree*/
#include <pcl/kdtree/kdtree_flann.h>

/*Immutable terrain contract indexed exactly like mapground*/
#include <perception_3d/terrain_model.h>

#include <cstdint>
#include <utility>

//@tf2::eigenToTransform
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>

namespace mpc_critics
{

class ModelSharedData{
  public:

    ModelSharedData(std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer):tf2Buffer_(m_tf2Buffer){
      pcl_perception_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      pcl_perception_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    };
    
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer(){return tf2Buffer_;}

    void updateData(){
      global_frame_ = robot_pose_.header.frame_id;
      base_frame_ = robot_pose_.child_frame_id;
      /* Critics get a fresh kd-tree after aggregateObservations in local planner. */
      pcl_perception_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
      if(pcl_perception_ && !pcl_perception_->points.empty()){
        pcl_perception_kdtree_->setInputCloud(pcl_perception_);        
      }
      
      pcl_prune_plan_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      for(auto i=prune_plan_.poses.begin();i!=prune_plan_.poses.end();i++){
        pcl::PointXYZI ipt;
        ipt.x = (*i).pose.position.x;
        ipt.y = (*i).pose.position.y;
        ipt.z = (*i).pose.position.z;
        ipt.intensity = 0.;
        pcl_prune_plan_->push_back(ipt);
      }
    }

    // Take a private, immutable-for-the-scoring-cycle copy of mapground.  The
    // perception callbacks may replace or mutate their PCL buffers after the
    // ground mutex is released; critics must never observe half of an update.
    void updateTerrainData(
      perception_3d::TerrainSnapshotConstPtr snapshot,
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ground,
      std::uint64_t ground_version)
    {
      if(!snapshot && !terrain_snapshot_ && terrain_ground_version_ == ground_version &&
        terrain_ground_ && terrain_ground_kdtree_)
      {
        return;
      }
      if(terrain_snapshot_ == snapshot && terrain_ground_version_ == ground_version &&
        terrain_ground_ && terrain_ground_kdtree_ && ground &&
        terrain_ground_->points.size() == ground->points.size())
      {
        return;
      }
      terrain_snapshot_ = std::move(snapshot);
      terrain_ground_version_ = ground_version;
      terrain_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      terrain_ground_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

      // Do not pay the full-map copy/KD-tree cost while the terrain feature is
      // unavailable.  An enabled TerrainSupportModel will reject the missing
      // snapshot before attempting a search.
      if(!terrain_snapshot_ || !ground || ground->points.empty()){
        return;
      }
      *terrain_ground_ = *ground;
      terrain_ground_kdtree_->setInputCloud(terrain_ground_);
    }

    void requestTerrainSupportData(){terrain_support_data_requested_ = true;}
    bool terrainSupportDataRequested() const{return terrain_support_data_requested_;}

    perception_3d::TerrainSnapshotConstPtr terrain_snapshot_;
    std::uint64_t terrain_ground_version_{0U};
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_ground_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr terrain_ground_kdtree_;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr pcl_perception_kdtree_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_perception_;
    
    //@ this will be easy use for kdtree from pct
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_prune_plan_;

    nav_msgs::msg::Path prune_plan_;

    geometry_msgs::msg::TransformStamped robot_pose_;

    std::string global_frame_, base_frame_;

    nav_msgs::msg::Odometry robot_state_;
    ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_state_;

    double heading_deviation_;

  private:

    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_; 
    bool terrain_support_data_requested_{false};
    

};


}//end of name space

#endif  // MODEL_SHARED_DATA_H
