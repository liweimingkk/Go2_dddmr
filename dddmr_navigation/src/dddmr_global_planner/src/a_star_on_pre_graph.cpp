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
#include <global_planner/a_star_on_pre_graph.h>

#include <limits>

AstarListPreGraph::AstarListPreGraph(perception_3d::StaticGraph& static_graph){
  static_graph_ = static_graph;
}

void AstarListPreGraph::setGraph(perception_3d::StaticGraph& static_graph){
  static_graph_ = static_graph;
}

void AstarListPreGraph::Initial(){
  as_list_.clear();
  graph_t* tmp_graph_t; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  tmp_graph_t = static_graph_.getGraphPtr();
  for(auto it=(*tmp_graph_t).begin();it!=(*tmp_graph_t).end();it++){
    NodePreGraph_t new_node = {.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=false, .is_opened=false};
    as_list_[(*it).first] = new_node;
  }
  f_priority_set_.clear();
}

NodePreGraph_t AstarListPreGraph::getNode(unsigned int node_index){

  return as_list_[node_index];
}

float AstarListPreGraph::getGVal(NodePreGraph_t& a_node){
  return as_list_[a_node.self_index].g;
}

void AstarListPreGraph::closeNode(NodePreGraph_t& a_node){
  as_list_[a_node.self_index].is_closed = true;
}

void AstarListPreGraph::updateNode(NodePreGraph_t& a_node){
  as_list_[a_node.self_index] = a_node;
  f_p_ afp;
  afp.first = a_node.f; //made minimum f to be top so we can pop it
  afp.second = a_node.self_index;
  f_priority_set_.insert(afp);
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

NodePreGraph_t AstarListPreGraph::getNode_wi_MinimumF(){
  // updateNode can leave stale entries behind. Drain them without dereferencing end().
  while (!f_priority_set_.empty()) {
    const auto first_it = f_priority_set_.begin();
    const NodePreGraph_t node = as_list_[first_it->second];
    f_priority_set_.erase(first_it);
    if (!node.is_closed) {
      return node;
    }
  }
  return NodePreGraph_t{
    std::numeric_limits<unsigned int>::max(), 0.0F, 0.0F, 0.0F,
    std::numeric_limits<unsigned int>::max(), true, false};
}

bool AstarListPreGraph::isClosed(unsigned int node_index){
  return as_list_[node_index].is_closed;
}

bool AstarListPreGraph::isOpened(unsigned int node_index){
  return as_list_[node_index].is_opened;
}

bool AstarListPreGraph::isFrontierEmpty(){
  return f_priority_set_.empty();
}

//@----------------------------------------------------------------------------------------

A_Star_on_PreGraph::A_Star_on_PreGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up, 
                                  perception_3d::StaticGraph& static_graph, 
                                  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
                                  double a_star_expanding_radius,
                                  const global_planner::TerrainEdgeValidatorConfig& terrain_edge_config)
  : terrain_edge_validator_(terrain_edge_config)
{
  static_graph_ = static_graph;
  perception_ros_ = perception_ros;
  a_star_expanding_radius_ = a_star_expanding_radius;
  turning_weight_ = 0.0;
  pc_original_z_up_ = pc_original_z_up;
  ASLS_ = new AstarListPreGraph(static_graph_);
}

A_Star_on_PreGraph::~A_Star_on_PreGraph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_PreGraph::updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up, 
                                  perception_3d::StaticGraph& static_graph){
  static_graph_ = static_graph;
  pc_original_z_up_ = pc_original_z_up;
  if (!ASLS_) {
    ASLS_ = new AstarListPreGraph(static_graph_);
  } else {
    ASLS_->setGraph(static_graph_);
  }
}

double A_Star_on_PreGraph::getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
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

bool A_Star_on_PreGraph::isLineOfSightClear(pcl::PointXYZI& pcl_current, pcl::PointXYZI& pcl_expanding, double inscribed_radius){

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

void A_Star_on_PreGraph::getPath(
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
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "Cannot plan on an invalid or stale pre-graph");
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
          rclcpp::get_logger("astar"), "Pre-graph A* planning binding changed: %s",
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
  NodePreGraph_t current_node = {.self_index=start, .g=0, .h=0, .f=f, .parent_index=start, .is_closed=false, .is_opened=true};

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
    RCLCPP_ERROR(rclcpp::get_logger("astar"), "Pre-graph A* cost configuration is invalid");
    log_terrain_statistics();
    return;
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
      RCLCPP_ERROR(
        rclcpp::get_logger("astar"), "Pre-graph frontier contains an invalid cost state");
      break;
    }
    //ROS_DEBUG("Expand node: %u", current_node.self_index);
    /*Get successors*/
    auto successors = static_graph_.getEdge(current_node.self_index);
    
    for(auto it = successors.begin(); it!=successors.end(); it++){

      if ((*it).first == current_node.self_index ||
        (*it).first >= pc_original_z_up_->points.size() ||
        !std::isfinite((*it).second) || (*it).second <= 0.0)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("astar"), "Pre-graph contains an invalid edge or edge cost");
        continue;
      }
      
      if((*it).second>a_star_expanding_radius_){
        continue;
      }
      //@ dGraphValue is the distance to lethal
      double dGraphValue = perception_ros_->get_min_dGraphValue((*it).first);

      const bool dynamic_obstacle =
        !std::isfinite(dGraphValue) || dGraphValue < inscribed_radius;
      float current_expanding_g = (*it).second;
      if (terrain_context.terrainEnabled()) {
        const auto terrain_validation = terrain_edge_validator_.evaluate(
          terrain_context, current_node.self_index, (*it).first,
          pc_original_z_up_->points[current_node.self_index],
          pc_original_z_up_->points[(*it).first], dynamic_obstacle);
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
        //ROS_DEBUG("%.2f,%.2f,%.2f, v: %.2f",pc_original_z_up_->points[(*it).first].x,pc_original_z_up_->points[(*it).first].y,pc_original_z_up_->points[(*it).first].z, dGraphValue);
        continue;
      }
      
      pcl::PointXYZI pcl_current = pc_original_z_up_->points[current_node.self_index];
      pcl::PointXYZI pcl_current_parent = pc_original_z_up_->points[current_node.parent_index];
      pcl::PointXYZI pcl_expanding = pc_original_z_up_->points[(*it).first];

      double factor = exp(-1.0 * inflation_descending_rate * (dGraphValue - inscribed_radius));

      //@ get current_parent, current, expanding to compute theta od expanding
      double theta = getThetaFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding);

      const float point_cost = pc_original_z_up_->points[current_node.self_index].intensity;
      if (!std::isfinite(factor) || factor < 0.0 || !std::isfinite(theta) || theta < 0.0 ||
        !std::isfinite(point_cost) || point_cost < 0.0F)
      {
        continue;
      }
      float new_g = current_node.g + current_expanding_g + factor * 1.0 +
                      theta*turning_weight_ + point_cost;

      float new_h = sqrt(pcl::geometry::squaredDistance(pcl_expanding, pcl_goal));
      float new_f = new_g + new_h;
      if (!std::isfinite(new_g) || new_g < 0.0F ||
        !std::isfinite(new_h) || new_h < 0.0F ||
        !std::isfinite(new_f) || new_f < 0.0F)
      {
        continue;
      }

      NodePreGraph_t new_node = {.self_index=((*it).first), .g=new_g, .h=new_h, .f=new_f, .parent_index=current_node.self_index, .is_closed=false, .is_opened=true};

      /*Check is in closed list*/
      if(ASLS_->isClosed((*it).first))
        continue;
      /*Check is in opened list*/
      else if(ASLS_->isOpened((*it).first)){
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
      //ROS_DEBUG("Found path");
      NodePreGraph_t trace_back = ASLS_->getNode(goal);
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
