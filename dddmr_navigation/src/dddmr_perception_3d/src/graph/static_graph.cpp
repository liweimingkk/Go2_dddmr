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
//@For graph
#include <perception_3d/static_graph.h>

namespace perception_3d
{

StaticGraph::StaticGraph(){
}

void StaticGraph::allocateGraph(unsigned long int m_size){
  clear();
  for(unsigned long int i=0; i<m_size; i++){
    node_penality_[i] = 0.0;
    edges_t my_edges;
    graph_[i] = my_edges;
  }
}

void StaticGraph::insertEdgeInNode(unsigned int node, edge_t& a_edge){
  if (graph_.count(node) > 0) {
    graph_[node].insert(a_edge);
  }
  else{
    //key is not exist, the initialization is required, we dont insert a key as omp will will be thread safe
  }
}

void StaticGraph::setPenality(unsigned int node, float penality){
  if (node_penality_.count(node) > 0) {
    if(penality>node_penality_[node])
      node_penality_[node] = penality;
  }
  else{
    //key is not exist, the initialization is required, we dont insert a key as omp will will be thread safe
  }
}

graph_t* StaticGraph::getGraphPtr(){
  return &graph_;
}

edges_t StaticGraph::getEdge(unsigned int node){
  const auto found = graph_.find(node);
  return found == graph_.end() ? edges_t{} : found->second;
}

float StaticGraph::getNodeWeight(unsigned int node){
  const auto found = node_penality_.find(node);
  return found == node_penality_.end() ? 0.0F : found->second;
}

void StaticGraph::clear(){
  node_penality_.clear();
  graph_.clear();
}

}
