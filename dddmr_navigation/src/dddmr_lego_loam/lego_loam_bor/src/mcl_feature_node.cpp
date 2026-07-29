#include "imageProjection.h"
#include "featureAssociation.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  // Localization is a live latest-state pipeline.  Do not let image
  // projection block behind feature association: that callback can briefly
  // wait for the external odometry callback running in the same executor.
  // A blocking single-slot channel can otherwise exhaust the executor and
  // permanently freeze segmented_cloud_pure.  Mapping keeps its blocking,
  // lossless channel in lego_loam_node.cpp.
  Channel<ProjectionOut> projection_out_channel(false);
  auto IP = std::make_shared<ImageProjection>("mcl_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("mcl_fa", projection_out_channel, association_out_channel);
  // Foxy may otherwise select too few workers for the image projection,
  // feature timer, odometry, and TF callback groups on embedded platforms.
  // Keep enough workers available so a time-aligned odometry wait cannot
  // starve the latest XT16 cloud callback.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4);
  executor.add_node(IP);
  executor.add_node(FA);
  IP->tfInitial();
  FA->tfInitial();
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
