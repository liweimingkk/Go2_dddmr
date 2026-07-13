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
#include <global_planner/a_star_on_pc.h>

#include <limits>

AstarList::AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up){
  updateGraph(pc_original_z_up);
}

void AstarList::updateGraph(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  if (pc_original_z_up_ && !pc_original_z_up_->empty()) {
    kdtree_ground_->setInputCloud(pc_original_z_up_);
  }
}

void AstarList::Initial(){
  as_list_.clear(); 
  for(unsigned int it=0; it!=pc_original_z_up_->points.size();it++){
    Node_t new_node = {.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=false, .is_opened=false};
    as_list_[it] = new_node;
  }
  f_priority_set_.clear();
}

Node_t AstarList::getNode(unsigned int node_index){

  return as_list_[node_index];
}

float AstarList::getGVal(Node_t& a_node){
  return as_list_[a_node.self_index].g;
}

void AstarList::closeNode(Node_t& a_node){
  as_list_[a_node.self_index].is_closed = true;
}

void AstarList::updateNode(Node_t& a_node){
  as_list_[a_node.self_index] = a_node;
  f_p_ afp;
  afp.first = a_node.f; //made minimum f to be top so we can pop it
  afp.second = a_node.self_index;
  f_priority_set_.insert(afp);
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

Node_t AstarList::getNode_wi_MinimumF(){
  // updateNode can leave stale entries behind. Drain them without dereferencing end().
  while (!f_priority_set_.empty()) {
    const auto first_it = f_priority_set_.begin();
    const Node_t node = as_list_[first_it->second];
    f_priority_set_.erase(first_it);
    if (!node.is_closed) {
      return node;
    }
  }
  return Node_t{
    std::numeric_limits<unsigned int>::max(), 0.0F, 0.0F, 0.0F,
    std::numeric_limits<unsigned int>::max(), true, false};
}

bool AstarList::isClosed(unsigned int node_index){
  return as_list_[node_index].is_closed;
}

bool AstarList::isOpened(unsigned int node_index){
  return as_list_[node_index].is_opened;
}

bool AstarList::isFrontierEmpty(){
  return f_priority_set_.empty();
}

//@----------------------------------------------------------------------------------------

A_Star_on_Graph::A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up, 
                                  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
                                  double a_star_expanding_radius,
                                  const global_planner::TerrainEdgeValidatorConfig& terrain_edge_config)
  : terrain_edge_validator_(terrain_edge_config)
{
  
  perception_ros_ = perception_ros;
  pc_original_z_up_ = pc_original_z_up;
  a_star_expanding_radius_ = a_star_expanding_radius;
  turning_weight_ = 0.0;
  ASLS_ = new AstarList(pc_original_z_up_);
}

A_Star_on_Graph::~A_Star_on_Graph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_Graph::updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  if (!ASLS_) {
    ASLS_ = new AstarList(pc_original_z_up_);
    return;
  }
  ASLS_->updateGraph(pc_original_z_up_);
}

double A_Star_on_Graph::getPitchFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1, s1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  s1 = sqrt(vx1*vx1 + vy1*vy1);
  //@ calculate vector: current -> expanding
  float vx2, vy2, s2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  s2 = sqrt(vx2*vx2 + vy2*vy2);

  float pitch = fabs(m_pcl_current_parent.z - m_pcl_expanding.z)/(s1+s2);

  return pitch;
}

double A_Star_on_Graph::getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  //@ calculate vector: current -> expanding
  float vx2, vy2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  float cos_theta = (vx1*vx2 + vy1*vy2)/(sqrt(vx1*vx1+vy1*vy1)*sqrt(vx2*vx2+vy2*vy2));
  if(fabs(cos_theta)>1)
    cos_theta = 1.0;
  double theta_of_vector = acos(cos_theta);
  if(vx1==0 && vy1==0)
    theta_of_vector = 0;
  else if(vx2==0 && vy2==0)
    theta_of_vector = 0;
  else if(fabs(fabs(vx1)-fabs(vx2))<=0.0001)
    theta_of_vector = 0;
  
  if(fabs(theta_of_vector)<=0.345)//cap
    theta_of_vector = 0.0;

  return theta_of_vector;
}

bool A_Star_on_Graph::isLineOfSightClear(pcl::PointXYZI& pcl_current, pcl::PointXYZI& pcl_expanding, double inscribed_radius){

  //@ generate line equation
  float dX =
      pcl_expanding.x - pcl_current.x;
  float dY =
      pcl_expanding.y - pcl_current.y;
  float dZ =
      pcl_expanding.z - pcl_current.z;
  
  float distance = sqrt(dX*dX + dY*dY + dZ*dZ);
  distance = distance/inscribed_radius; //sample by every inscribed radius
  float dt = 1/distance;
  for(float t=0; t<=1.0+dt; t+=dt){
    float r = t;
    if(t>=1.0) //@ make sure we examine t=1.0
      r = 1.0;
    pcl::PointXYZI a_pt;
    a_pt.intensity = 0.0;
    a_pt.x = pcl_current.x + dX*r;
    a_pt.y = pcl_current.y + dY*r;
    a_pt.z = pcl_current.z + dZ*r;
    std::vector<int> pidx;
    std::vector<float> prsd;
    kdtree_lethal_->radiusSearch(a_pt, 2*inscribed_radius, pidx, prsd);
    if(pidx.size()>1){
      return false;
    }
  }
  return true;
}

void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path,
  const global_planner::planner_safety::PlanningDataBinding * expected_binding,
  global_planner::TerrainEdgeRejectionStatistics * terrain_statistics){

  path.clear();
  if (terrain_statistics != nullptr) {
    *terrain_statistics = global_planner::TerrainEdgeRejectionStatistics{};
  }
  if (!pc_original_z_up_ || pc_original_z_up_->empty() || !ASLS_ || !perception_ros_ ||
    start >= pc_original_z_up_->points.size() || goal >= pc_original_z_up_->points.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "Cannot plan on an invalid or stale graph");
    return;
  }

  const auto shared_data = perception_ros_->getSharedDataPtr();
  if (!shared_data) {
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "Cannot plan without shared perception data");
    return;
  }
  std::unique_lock<std::recursive_mutex> perception_lock(
    shared_data->ground_kdtree_cb_mutex_);

  auto binding_is_current = [&]() {
      if (expected_binding == nullptr) {
        return true;
      }
      const auto observed = global_planner::capturePlanningDataBinding(
        shared_data, expected_binding->terrain_enabled);
      std::string rejection;
      if (!global_planner::planner_safety::planningBindingsMatch(
          *expected_binding, observed, &rejection))
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("astar"), "A* planning binding changed: %s",
          rejection.c_str());
        return false;
      }
      return true;
    };
  if (!binding_is_current()) {
    return;
  }

  auto terrain_context = terrain_edge_validator_.beginSearch(shared_data);
  auto log_terrain_statistics = [&]() {
      if (terrain_statistics != nullptr) {
        *terrain_statistics = terrain_context.statistics();
      }
      if (terrain_context.terrainEnabled()) {
        RCLCPP_INFO(
          rclcpp::get_logger("astar"), "%s",
          terrain_context.statistics().toStructuredString().c_str());
      }
    };
  if (terrain_context.terrainEnabled() && !terrain_context.ready()) {
    log_terrain_statistics();
    return;
  }
  if (!terrain_edge_validator_.validateEndpoint(
      terrain_context, start, pc_original_z_up_->points[start]) ||
    !terrain_edge_validator_.validateEndpoint(
      terrain_context, goal, pc_original_z_up_->points[goal]))
  {
    log_terrain_statistics();
    return;
  }

  //RCLCPP_INFO(rclcpp::get_logger("astar"),"Start: %u, Goal: %u", start, goal);

  /*
  Create the first node which is start and add into frontier
  */

  pcl::PointXYZI pcl_goal = pc_original_z_up_->points[goal];
  pcl::PointXYZI pcl_start = pc_original_z_up_->points[start];
  float f = sqrt(pcl::geometry::squaredDistance(pcl_start, pcl_goal));
  Node_t current_node = {.self_index=start, .g=0, .h=0, .f=f, .parent_index=start, .is_closed=false, .is_opened=true};

  ASLS_->Initial();
  ASLS_->updateNode(current_node);
  
  double inscribed_radius = perception_ros_->getGlobalUtils()->getInscribedRadius();
  double inflation_descending_rate = perception_ros_->getGlobalUtils()->getInflationDescendingRate();
  double max_obstacle_distance = perception_ros_->getGlobalUtils()->getMaxObstacleDistance();
  if (!std::isfinite(a_star_expanding_radius_) || a_star_expanding_radius_ <= 0.0 ||
    !std::isfinite(turning_weight_) || turning_weight_ < 0.0 ||
    !std::isfinite(inscribed_radius) || inscribed_radius <= 0.0 ||
    !std::isfinite(inflation_descending_rate) || inflation_descending_rate < 0.0 ||
    !std::isfinite(max_obstacle_distance) || max_obstacle_distance < inscribed_radius)
  {
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "A* cost configuration is invalid");
    log_terrain_statistics();
    return;
  }
  
  perception_ros_->getStackedPerception()->aggregateLethal();
  if (!shared_data->aggregate_lethal_ || !shared_data->sGraph_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "A* perception cost inputs are missing");
    log_terrain_statistics();
    return;
  }
  //@ generate kd-tree and handle no point cloud edge case
  kdtree_lethal_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  
  if(shared_data->aggregate_lethal_->points.size()>0){
    kdtree_lethal_->setInputCloud(shared_data->aggregate_lethal_);
  }
    

  while(!ASLS_->isFrontierEmpty()){ 
    /*Pop minimum F, we leverage prior queue, so we dont need to loop frontier everytime*/
    current_node = ASLS_->getNode_wi_MinimumF();
    if (current_node.self_index >= pc_original_z_up_->points.size()) {
      break;
    }
    if (current_node.parent_index >= pc_original_z_up_->points.size() ||
      !std::isfinite(current_node.g) || current_node.g < 0.0F)
    {
      RCLCPP_ERROR(rclcpp::get_logger("astar"), "A* frontier contains an invalid cost state");
      break;
    }

    //RCLCPP_INFO(rclcpp::get_logger("astar"), "Expand node: %u", current_node.self_index);
    /*Get successors*/
    pcl::PointXYZI pcl_now = pc_original_z_up_->points[current_node.self_index];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ASLS_->kdtree_ground_->radiusSearch(pcl_now, a_star_expanding_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    //@dealing with orphan node
    if(pointIdxRadiusSearch.size()<8){
      std::vector<int> pointIdxRadiusX2Search;
      std::vector<float> pointRadiusSquaredDistanceX2;
      //ASLS_->kdtree_ground_->nearestKSearch(pcl_now, 8, pointIdxRadiusX2Search, pointRadiusSquaredDistanceX2);
      ASLS_->kdtree_ground_->radiusSearch(pcl_now, 2*a_star_expanding_radius_, pointIdxRadiusX2Search, pointRadiusSquaredDistanceX2);
      pointIdxRadiusSearch.swap(pointIdxRadiusX2Search);
      pointRadiusSquaredDistance.swap(pointRadiusSquaredDistanceX2);
    }

    if(pointIdxRadiusSearch.empty() || pointIdxRadiusSearch.size() != pointRadiusSquaredDistance.size()){
      RCLCPP_ERROR(rclcpp::get_logger("astar"),
        "Invalid radius-search result at node %u: indices=%zu distances=%zu",
        current_node.self_index, pointIdxRadiusSearch.size(), pointRadiusSquaredDistance.size());
      ASLS_->closeNode(current_node);
      continue;
    }

    //@ calculated average intensity, because we have sparse low cost orphan, and it is unlikely to have a low cost node surrounded by high cost nodes
    float avg_intensity = 0.0F;
    bool invalid_neighbor_cost = false;
    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      const int neighbor_index = pointIdxRadiusSearch[it];
      const float squared_distance = pointRadiusSquaredDistance[it];
      if (neighbor_index < 0 ||
        static_cast<std::size_t>(neighbor_index) >= pc_original_z_up_->points.size() ||
        !std::isfinite(squared_distance) || squared_distance < 0.0F)
      {
        invalid_neighbor_cost = true;
        break;
      }
      const float intensity = pc_original_z_up_->points[neighbor_index].intensity;
      if (!std::isfinite(intensity) || intensity < 0.0F) {
        invalid_neighbor_cost = true;
        break;
      }
      avg_intensity += intensity;
    }
    if (invalid_neighbor_cost || !std::isfinite(avg_intensity)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("astar"), "A* neighbor search returned an invalid edge cost");
      ASLS_->closeNode(current_node);
      continue;
    }
    avg_intensity = avg_intensity/pointIdxRadiusSearch.size();

    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      
      const int expanding_index = pointIdxRadiusSearch[it];
      if (expanding_index < 0) {
        continue;
      }
      unsigned int current_expanding_index = static_cast<unsigned int>(expanding_index);
      if (current_expanding_index == current_node.self_index ||
        current_expanding_index >= pc_original_z_up_->points.size() ||
        !std::isfinite(pointRadiusSquaredDistance[it]) ||
        pointRadiusSquaredDistance[it] <= 0.0F)
      {
        continue;
      }
      const float geometric_edge_length = sqrt(pointRadiusSquaredDistance[it]);
      float current_expanding_g = geometric_edge_length;

      //@ dGraphValue is the distance to lethal
      double dGraphValue = perception_ros_->get_min_dGraphValue(current_expanding_index);

      const bool dynamic_obstacle =
        !std::isfinite(dGraphValue) || dGraphValue < inscribed_radius;
      if (terrain_context.terrainEnabled()) {
        const auto terrain_validation = terrain_edge_validator_.evaluate(
          terrain_context, current_node.self_index, current_expanding_index,
          pc_original_z_up_->points[current_node.self_index],
          pc_original_z_up_->points[current_expanding_index], dynamic_obstacle);
        if (!terrain_validation.policy_result.accepted) {
          continue;
        }
        current_expanding_g = terrain_validation.policy_result.traversal_cost;
      }
      if (!std::isfinite(current_expanding_g) || current_expanding_g < 0.0F) {
        continue;
      }

      /*This is for lethal*/
      if(dynamic_obstacle){
        //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "%.2f,%.2f,%.2f, v: %.2f",pc_original_z_up_->points[(*it).first].x,pc_original_z_up_->points[(*it).first].y,pc_original_z_up_->points[(*it).first].z, dGraphValue);
        continue;
      }

      pcl::PointXYZI pcl_current = pc_original_z_up_->points[current_node.self_index];
      pcl::PointXYZI pcl_current_parent = pc_original_z_up_->points[current_node.parent_index];
      pcl::PointXYZI pcl_expanding = pc_original_z_up_->points[current_expanding_index];

      //@ check line-of-sight when distance is 2 times larger than inscribed_radius
      if(geometric_edge_length>=2*inscribed_radius){
        if(!isLineOfSightClear(pcl_current, pcl_expanding, inscribed_radius))
          continue;
      }
      
      double factor = exp(-1.0 * inflation_descending_rate * (dGraphValue - inscribed_radius));

      //@ get current_parent, current, expanding to compute theta od expanding
      double theta = getThetaFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding);
      
      //if(getPitchFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding)>0.2)
      //  continue;
      
      float ground_edge_weight = avg_intensity;
      float node_weight = shared_data->sGraph_ptr_->getNodeWeight(current_expanding_index);
      if (!std::isfinite(factor) || factor < 0.0 || !std::isfinite(theta) || theta < 0.0 ||
        !std::isfinite(node_weight) || node_weight < 0.0F)
      {
        continue;
      }
      float new_g = current_node.g + current_expanding_g + factor * 1.0 + node_weight + theta*turning_weight_ + ground_edge_weight;
      float new_h = sqrt(pcl::geometry::squaredDistance(pcl_expanding, pcl_goal));
      float new_f = new_g + new_h;
      if (!std::isfinite(new_g) || new_g < 0.0F ||
        !std::isfinite(new_h) || new_h < 0.0F ||
        !std::isfinite(new_f) || new_f < 0.0F)
      {
        continue;
      }

      Node_t new_node = {.self_index=(current_expanding_index), .g=new_g, .h=new_h, .f=new_f, .parent_index=current_node.self_index, .is_closed=false, .is_opened=true};

      /*Check is in closed list*/
      if(ASLS_->isClosed(current_expanding_index))
        continue;
      /*Check is in opened list*/
      else if(ASLS_->isOpened(current_expanding_index)){
        if(ASLS_->getGVal(new_node)>new_g){
          ASLS_->updateNode(new_node);          
        }
      }
      /*addNode*/
      else{
        ASLS_->updateNode(new_node);
      }
        
      
    }

    /*Close this node*/
    ASLS_->closeNode(current_node);

    /*If goal is in closed list, we are done*/
    if(ASLS_->isClosed(goal)){
      Node_t trace_back = ASLS_->getNode(goal);
      while(trace_back.self_index!=trace_back.parent_index){
        path.push_back(trace_back.self_index);
        trace_back = ASLS_->getNode(trace_back.parent_index);
      }
      path.push_back(trace_back.self_index);//Push start point
      std::reverse(path.begin(),path.end()); 
      break;
    }

    /*Check if*/
  }

  log_terrain_statistics();

  if (!binding_is_current()) {
    path.clear();
  }

}
