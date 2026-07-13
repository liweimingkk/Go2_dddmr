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
#include <perception_3d/cluster_marking.h>

#include <cmath>


namespace perception_3d
{

Marking::~Marking(){
  //@ loop marking_ map to reset all ptr
  for(auto ix=marking_.begin(); ix!=marking_.end();ix++){
    for(auto iy=(*ix).second.begin(); iy!=(*ix).second.end();iy++){
      for(auto iz=(*iy).second.begin(); iz!=(*iy).second.end();iz++){
        (*iz).second.pc_.reset();
        (*iz).second.mc_.reset();
      } 
    }
  }
}

void Marking::computeMinDistanceFromObstacle2GroundNodes(  
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr,
  std::unordered_map<int, float>& nodes_of_min_distance){

  if(!shared_data_){
    return;
  }
  // Some callers already hold this recursive ground lock.  Reacquiring it is
  // intentional and guarantees direct/unit-test callers receive the same
  // immutable ground/KD-tree contract.  Terrain identity is acquired second,
  // matching StaticLayer's update order.
  std::unique_lock<std::recursive_mutex> ground_lock(
    shared_data_->ground_kdtree_cb_mutex_);
  std::unique_lock<std::mutex> terrain_identity_lease;
  if(stair_riser_semantics_.enabled){
    terrain_identity_lease = shared_data_->acquireTerrainIdentityLease();
  }
  (void)terrain_identity_lease;

  pcl::PointCloud<pcl::PointXYZI>::Ptr marking_cloud = pcptr;
  const auto scoring_snapshot = stair_riser_semantics_.enabled && shared_data_ ?
    shared_data_->getTerrainSnapshot() : TerrainSnapshotConstPtr{};
  std::string config_error;
  const bool semantic_filter_ready =
    stair_riser_semantics_.enabled && clock_ && shared_data_ && scoring_snapshot &&
    shared_data_->pcl_ground_ && shared_data_->kdtree_ground_ &&
    scoring_snapshot->nodes().size() == shared_data_->pcl_ground_->size() &&
    std::isfinite(terrain_surface_plane_tolerance_m_) &&
    terrain_surface_plane_tolerance_m_ > 0.0 &&
    StairRiserSemantics::validConfig(stair_riser_semantics_, &config_error);
  if(stair_riser_semantics_.enabled && !semantic_filter_ready && clock_){
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(name_), *clock_, 2000,
      "stair_riser_marking_fail_closed: semantic prerequisites unavailable (%s); "
      "retaining every cluster point as an obstacle",
      config_error.empty() ? "snapshot/ground unavailable" : config_error.c_str());
  }
  if(semantic_filter_ready){
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    filtered->header = pcptr->header;
    filtered->reserve(pcptr->size());
    std::size_t passthrough_count = 0U;
    for(const auto& point : pcptr->points){
      std::vector<int> terrain_indices(1);
      std::vector<float> squared_distances(1);
      StairRiserSemanticResult classification;
      if(shared_data_->kdtree_ground_->nearestKSearch(
          point, 1, terrain_indices, squared_distances) > 0 &&
        terrain_indices.front() >= 0 &&
        static_cast<std::size_t>(terrain_indices.front()) < shared_data_->pcl_ground_->size())
      {
        const std::size_t terrain_index =
          static_cast<std::size_t>(terrain_indices.front());
        const auto& terrain_point = shared_data_->pcl_ground_->points[terrain_index];
        StairRiserObservation observation;
        observation.snapshot = scoring_snapshot;
        observation.terrain_ground_version = scoring_snapshot->version();
        observation.now_nanoseconds = clock_->now().nanoseconds();
        observation.terrain_node_index = terrain_index;
        observation.terrain_node_position = Eigen::Vector3f(
          terrain_point.x, terrain_point.y, terrain_point.z);
        observation.obstacle_position = Eigen::Vector3f(point.x, point.y, point.z);
        // No temporal dynamic label is present in the legacy cluster point
        // type.  This is "not confirmed dynamic", not proof of static state.
        // Non-coincident geometry stays lethal; a perfectly co-planar return
        // cannot be distinguished until a tracker supplies explicit evidence.
        observation.dynamic_obstacle_confirmed = false;
        classification = StairRiserSemantics::classify(
          stair_riser_semantics_, observation);
      }
      if(classification.expected_riser){
        ++passthrough_count;
      }else{
        filtered->push_back(point);
      }
    }
    // Never apply a classification computed against a snapshot that was
    // replaced while iterating.  The original cluster remains untouched and
    // is used for fail-closed marking in that case.
    if(shared_data_->getTerrainSnapshot() == scoring_snapshot){
      marking_cloud = std::move(filtered);
      if(passthrough_count > 0U){
        RCLCPP_DEBUG_THROTTLE(
          rclcpp::get_logger(name_), *clock_, 1000,
          "stair_riser_marking_passthrough points=%zu retained_obstacles=%zu "
          "(raw cluster retained for debug)",
          passthrough_count, marking_cloud->size());
      }
    }
  }
  const bool semantic_association_ready = semantic_filter_ready &&
    shared_data_->getTerrainSnapshot() == scoring_snapshot;

  if(marking_cloud->empty()){
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (marking_cloud);
  proj.setModelCoefficients (pcplaneptr);
  proj.filter (*projected_cloud_cluster);

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (projected_cloud_cluster);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*projected_cloud_cluster);

  for(auto prj_pt_it=projected_cloud_cluster->points.begin();prj_pt_it!=projected_cloud_cluster->points.end();prj_pt_it++){
    pcl::PointXYZI pt;
    pt.x = (*prj_pt_it).x;
    pt.y = (*prj_pt_it).y;
    pt.z = (*prj_pt_it).z;

    std::vector<int> id_tmp;
    std::vector<float> sqdist_tmp;
    const TerrainNode* reference_node = nullptr;
    pcl::PointXYZI reference_ground_point;
    if(semantic_association_ready){
      std::vector<int> reference_indices(1);
      std::vector<float> reference_squared_distances(1);
      if(shared_data_->kdtree_ground_->nearestKSearch(
          pt, 1, reference_indices, reference_squared_distances) > 0 &&
        reference_indices.front() >= 0 &&
        static_cast<std::size_t>(reference_indices.front()) <
        shared_data_->pcl_ground_->size())
      {
        const std::size_t reference_index =
          static_cast<std::size_t>(reference_indices.front());
        reference_node = scoring_snapshot->nodeAt(reference_index);
        reference_ground_point = shared_data_->pcl_ground_->points[reference_index];
        if(!reference_node || reference_node->surface_id < 0 ||
          !reference_node->normal.allFinite() ||
          reference_node->normal.norm() <= 1.0e-6F)
        {
          reference_node = nullptr;
        }
      }
    }
    //@ We mark lethal
    if(shared_data_->kdtree_ground_->radiusSearch(pt, inflation_radius_, id_tmp, sqdist_tmp)){
      for(int i=0;i<id_tmp.size();i++){
        if(semantic_association_ready && reference_node){
          const std::size_t candidate_index = static_cast<std::size_t>(id_tmp[i]);
          const TerrainNode* candidate_node = scoring_snapshot->nodeAt(candidate_index);
          if(!candidate_node){
            continue;
          }
          const auto& candidate_ground = shared_data_->pcl_ground_->points[candidate_index];
          if(!terrainSurfaceProjectionCompatible(
              *reference_node,
              Eigen::Vector3f(
                reference_ground_point.x, reference_ground_point.y,
                reference_ground_point.z),
              *candidate_node,
              Eigen::Vector3f(
                candidate_ground.x, candidate_ground.y, candidate_ground.z),
              static_cast<float>(terrain_surface_plane_tolerance_m_)))
          {
            continue;
          }
        }
        float dx = pt.x - shared_data_->pcl_ground_->points[id_tmp[i]].x;
        float dy = pt.y - shared_data_->pcl_ground_->points[id_tmp[i]].y;
        float dz = pt.z - shared_data_->pcl_ground_->points[id_tmp[i]].z;
        //@ Remove z value (see issue 8), we might have to review this assumption.
        float distance = sqrt(dx*dx + dy* dy);
        //RCLCPP_DEBUG(rclcpp::get_logger("cluster_marking"),"Distance xyz: %.2f, distance xy: %.2f", sqrt(sqdist_tmp[i]), distance);
        //if(nodes_of_min_distance.insert(std::make_pair(id_tmp[i], sqrt(sqdist_tmp[i]))).second == false)
        if(nodes_of_min_distance.insert(std::make_pair(id_tmp[i], distance)).second == false)
        {
          //@ key was presented
          //nodes_of_min_distance[id_tmp[i]] = std::min(nodes_of_min_distance[id_tmp[i]], sqrt(sqdist_tmp[i]));
          nodes_of_min_distance[id_tmp[i]] = std::min(nodes_of_min_distance[id_tmp[i]], distance);
        }

      }
      
    }
  }

}


void Marking::addPCPtr(const double cx, const double cy, const double cz, 
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr){
  
  int x = cx/xy_resolution_;
  int y = cy/xy_resolution_;
  int z = cz/height_resolution_;
  if(marking_[x][y][z].pc_!= nullptr){
    marking_[x][y][z].pc_.reset();
    marking_[x][y][z].mc_.reset();
  }
  marking_[x][y][z].pc_ = pcptr;
  marking_[x][y][z].mc_ = pcplaneptr;
  std::unordered_map<int, float> nodes_of_min_distance;
  computeMinDistanceFromObstacle2GroundNodes(pcptr, pcplaneptr, nodes_of_min_distance);
  marking_[x][y][z].nodes_of_min_distance_ = nodes_of_min_distance;
  
  for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
    //RCLCPP_INFO(rclcpp::get_logger("ClusterMarking"), "%.2f", (*id).second);
    dGraph_->setValue((*id).first, (*id).second);
    if((*id).second<=inscribed_radius_)
      lethal_map_[(*id).first] = (*id).second;
  }
  
}

void Marking::removePCPtr(perception_3d::per_marking& per_marking){
  
  auto nodes_of_min_distance = per_marking.nodes_of_min_distance_;

  for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
    dGraph_->clearValue((*id).first, 9999.0);
    if((*id).second<=inscribed_radius_)
      lethal_map_.erase((*id).first);
  }

  per_marking.pc_.reset();
  per_marking.mc_.reset();

}

void Marking::updateCleared(const std::vector<marking_voxel>& current_observation_ptr){
  /*
  for(auto it=current_observation_ptr.begin();it!=current_observation_ptr.end();it++){
    auto pcptr = marking_[(*it).x][(*it).y][(*it).z].first;
    auto pcplaneptr = marking_[(*it).x][(*it).y][(*it).z].second;

    //@Filter out empty cloud
    if(pcptr->points.empty())
      continue;

    std::unordered_map<int, float> nodes_of_min_distance;
    computeMinDistanceFromObstacle2GroundNodes(pcptr, pcplaneptr, nodes_of_min_distance);
    for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
      dGraph_->setValue((*id).first,(*id).second);
    }
  }
  */
}

}//end of name space
