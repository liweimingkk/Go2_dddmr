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
#include "perception_3d/shared_data.h"

#include <limits>
#include <utility>

namespace perception_3d
{

SharedData::SharedData(){
  is_static_layer_ready_ = false;
  static_map_size_ = 0;
  static_ground_size_ = 0;
  aggregate_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  current_allowed_max_linear_speed_ = -1.0;
}

void SharedData::setTerrainSnapshot(TerrainSnapshotConstPtr snapshot){
  const std::lock_guard<std::mutex> lock(terrain_identity_mutex_);
  if (static_update_pending_ && snapshot) {
    return;
  }
  std::atomic_store_explicit(
    &terrain_snapshot_, std::move(snapshot), std::memory_order_release);
}

TerrainSnapshotConstPtr SharedData::getTerrainSnapshot() const{
  return std::atomic_load_explicit(&terrain_snapshot_, std::memory_order_acquire);
}

std::unique_lock<std::mutex> SharedData::acquireTerrainIdentityLease() const{
  return std::unique_lock<std::mutex>(terrain_identity_mutex_);
}

std::uint64_t SharedData::invalidateTerrainForStaticGroundUpdate(){
  const auto token = beginStaticMapGroundUpdate();
  return commitStaticMapGroundUpdate(token);
}

std::uint64_t SharedData::beginStaticMapGroundUpdate(){
  const std::lock_guard<std::mutex> lock(terrain_identity_mutex_);

  std::atomic_store_explicit(
    &terrain_snapshot_, TerrainSnapshotConstPtr{}, std::memory_order_release);
  static_update_pending_ = true;
  if (static_update_token_ != std::numeric_limits<std::uint64_t>::max()) {
    ++static_update_token_;
  }
  return static_update_token_;
}

std::uint64_t SharedData::commitStaticMapGroundUpdate(
  const std::uint64_t expected_update_token)
{
  const std::lock_guard<std::mutex> lock(terrain_identity_mutex_);
  if (!static_update_pending_ || expected_update_token == 0U ||
    expected_update_token != static_update_token_)
  {
    return 0U;
  }
  const auto generation = static_ground_generation_.load(std::memory_order_acquire);
  if (generation == std::numeric_limits<std::uint64_t>::max()) {
    return 0U;
  }
  static_update_pending_ = false;
  static_ground_generation_.store(generation + 1U, std::memory_order_release);
  return generation + 1U;
}

bool SharedData::isStaticMapGroundUpdatePending() const{
  const std::lock_guard<std::mutex> lock(terrain_identity_mutex_);
  return static_update_pending_;
}

std::uint64_t SharedData::getStaticGroundGeneration() const noexcept{
  return static_ground_generation_.load(std::memory_order_acquire);
}

bool SharedData::setTerrainSnapshotForStaticGroundGeneration(
  TerrainSnapshotConstPtr snapshot,
  const std::uint64_t expected_generation)
{
  const std::lock_guard<std::mutex> lock(terrain_identity_mutex_);
  if(static_update_pending_ || expected_generation == 0U ||
    static_ground_generation_.load(std::memory_order_acquire) != expected_generation)
  {
    return false;
  }

  std::atomic_store_explicit(
    &terrain_snapshot_, std::move(snapshot), std::memory_order_release);
  return true;
}

void SharedData::requestAllLayersToResetDGraph(){
  for(auto i=dgraph_update_request_.begin();i!=dgraph_update_request_.end();i++){
    (*i).second = true;
  }
}

bool SharedData::isAllLayersBeenReset(){
  for(auto i=dgraph_update_request_.begin();i!=dgraph_update_request_.end();i++){
    if((*i).second)
      return false;
  }
  return true;
}

}//end of name space
