#include <chrono>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dddmr_pg_map_server.h"
#include "dddmr_sys_core/srv/get_key_frame_cloud.hpp"

namespace
{
using namespace std::chrono_literals;
using GetKeyFrameCloud = dddmr_sys_core::srv::GetKeyFrameCloud;
using PointCloud = pcl::PointCloud<dddmr_pg_map_server::pcl_t>;

class PGMapServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    test_dir_ = std::filesystem::temp_directory_path() /
      ("dddmr_pg_map_server_test_" + std::to_string(test_number_++));
    std::filesystem::create_directories(test_dir_ / "pcd");
    writePoseGraph();
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove_all(test_dir_, error);
  }

  void writePoseGraph()
  {
    pcl::PointCloud<PointTypePose> poses;
    for (int i = 0; i < 3; ++i) {
      PointTypePose pose{};
      pose.x = static_cast<float>(i * 10);
      poses.push_back(pose);
    }
    ASSERT_EQ(
      pcl::io::savePCDFileBinary((test_dir_ / "poses.pcd").string(), poses),
      0);

    for (int i = 0; i < 3; ++i) {
      PointCloud feature;
      PointCloud surface;
      PointCloud ground;
      if (i != 1) {
        dddmr_pg_map_server::pcl_t point{};
        point.x = 1.0F;
        feature.push_back(point);
        surface.push_back(point);
        if (i == 0) {
          point.x = 0.05F;
          ground.push_back(point);
          point.x = 0.15F;
          ground.push_back(point);
        } else {
          ground.push_back(point);
        }
        ASSERT_EQ(
          pcl::io::savePCDFileBinary(
            (test_dir_ / "pcd" / (std::to_string(i) + "_feature.pcd")).string(),
            feature),
          0);
        ASSERT_EQ(
          pcl::io::savePCDFileBinary(
            (test_dir_ / "pcd" / (std::to_string(i) + "_surface.pcd")).string(),
            surface),
          0);
        ASSERT_EQ(
          pcl::io::savePCDFileBinary(
            (test_dir_ / "pcd" / (std::to_string(i) + "_ground.pcd")).string(),
            ground),
          0);
      }
    }
  }

  void writeSurfaceMergePoseGraph(bool include_ground)
  {
    std::filesystem::remove_all(test_dir_ / "pcd");
    std::filesystem::create_directories(test_dir_ / "pcd");

    pcl::PointCloud<PointTypePose> poses;
    PointTypePose pose{};
    poses.push_back(pose);
    ASSERT_EQ(
      pcl::io::savePCDFileBinary((test_dir_ / "poses.pcd").string(), poses),
      0);

    PointCloud ground;
    PointCloud surface;
    for (int i = 0; i < 12; ++i) {
      dddmr_pg_map_server::pcl_t point{};
      point.x = static_cast<float>(i) * 0.02F;
      if (include_ground) {
        ground.push_back(point);
        surface.push_back(point);
      }
      point.z = 0.5F;
      surface.push_back(point);
    }
    if (include_ground) {
      ASSERT_EQ(
        pcl::io::savePCDFileBinary(
          (test_dir_ / "pcd" / "0_ground.pcd").string(), ground),
        0);
    }
    ASSERT_EQ(
      pcl::io::savePCDFileBinary(
        (test_dir_ / "pcd" / "0_surface.pcd").string(), surface),
      0);
  }

  std::shared_ptr<dddmr_pg_map_server::DDDMRPGMapServer> makeServer(
    const std::string & name, double map_voxel,
    const std::vector<rclcpp::Parameter> & extra_parameters = {}) const
  {
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("pose_graph_dir", test_dir_.string()),
      rclcpp::Parameter("complete_map_voxel_size", map_voxel)};
    parameters.insert(
      parameters.end(), extra_parameters.begin(), extra_parameters.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return std::make_shared<dddmr_pg_map_server::DDDMRPGMapServer>(
      name, options);
  }

  GetKeyFrameCloud::Response::SharedPtr requestKeyFrame(
    const std::shared_ptr<dddmr_pg_map_server::DDDMRPGMapServer> & server,
    int32_t keyframe_number) const
  {
    auto client_node = std::make_shared<rclcpp::Node>("pg_map_server_test_client");
    auto client = client_node->create_client<GetKeyFrameCloud>(
      "/" + std::string(server->get_name()) + "/get_key_frame_cloud");
    auto request = std::make_shared<GetKeyFrameCloud::Request>();
    request->key_frame_number = keyframe_number;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    executor.add_node(client_node);
    EXPECT_TRUE(client->wait_for_service(1s));
    auto future = client->async_send_request(request);
    EXPECT_EQ(
      executor.spin_until_future_complete(future, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    executor.remove_node(client_node);
    executor.remove_node(server);
    return future.get();
  }

  sensor_msgs::msg::PointCloud2::SharedPtr receiveLatchedCloud(
    const std::shared_ptr<dddmr_pg_map_server::DDDMRPGMapServer> & server,
    const std::string & topic_suffix) const
  {
    auto client_node = std::make_shared<rclcpp::Node>(
      std::string(server->get_name()) + "_test_subscriber");
    sensor_msgs::msg::PointCloud2::SharedPtr message;
    auto subscription = client_node->create_subscription<
      sensor_msgs::msg::PointCloud2>(
      "/" + std::string(server->get_name()) + "/" + topic_suffix,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [&message](sensor_msgs::msg::PointCloud2::SharedPtr received) {
        message = std::move(received);
      });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    executor.add_node(client_node);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!message && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    executor.remove_node(client_node);
    executor.remove_node(server);
    return message;
  }

  std::filesystem::path test_dir_;
  inline static int test_number_{0};
};

TEST_F(PGMapServerTest, GroundVoxelDefaultsToCompleteMapVoxel)
{
  auto server = makeServer("fallback_map", 0.17);
  EXPECT_NEAR(
    server->get_parameter("complete_ground_voxel_size").as_double(), 0.17,
    1e-6);
}

TEST_F(PGMapServerTest, GroundVoxelCanBeConfiguredIndependently)
{
  auto server = makeServer(
    "independent_map", 0.2,
    {rclcpp::Parameter("complete_ground_voxel_size", 0.1)});
  EXPECT_DOUBLE_EQ(
    server->get_parameter("complete_map_voxel_size").as_double(), 0.2);
  EXPECT_DOUBLE_EQ(
    server->get_parameter("complete_ground_voxel_size").as_double(), 0.1);
}

TEST_F(PGMapServerTest, RejectsInvalidMapVoxelSize)
{
  EXPECT_THROW(
    makeServer("invalid_map_voxel", 0.0),
    std::invalid_argument);
}

TEST_F(PGMapServerTest, RejectsInvalidGroundVoxelSize)
{
  EXPECT_THROW(
    makeServer(
      "invalid_ground_voxel", 0.2,
      {rclcpp::Parameter(
          "complete_ground_voxel_size",
          std::numeric_limits<double>::quiet_NaN())}),
    std::invalid_argument);
}

TEST_F(PGMapServerTest, RejectsInvalidSurfaceGroundExclusionRadius)
{
  EXPECT_THROW(
    makeServer(
      "invalid_surface_ground_radius", 0.2,
      {rclcpp::Parameter("surface_ground_exclusion_radius", 0.0)}),
    std::invalid_argument);
}

TEST_F(PGMapServerTest, GroundMapUsesIndependentVoxelSize)
{
  auto server = makeServer(
    "ground_voxel_map", 0.2,
    {rclcpp::Parameter("complete_ground_voxel_size", 0.1)});
  auto message = receiveLatchedCloud(server, "mapground");
  ASSERT_NE(message, nullptr);
  EXPECT_EQ(
    static_cast<std::size_t>(message->width) * message->height, 3U);
}

TEST_F(PGMapServerTest, MergesSurfaceButExcludesGroundOverlap)
{
  writeSurfaceMergePoseGraph(true);
  auto server = makeServer(
    "surface_merge_map", 0.005,
    {
      rclcpp::Parameter("merge_non_ground_surface_into_mapcloud", true),
      rclcpp::Parameter("surface_ground_exclusion_radius", 0.05)
    });
  auto message = receiveLatchedCloud(server, "mapcloud");
  ASSERT_NE(message, nullptr);
  PointCloud cloud;
  pcl::fromROSMsg(*message, cloud);
  ASSERT_EQ(cloud.size(), 12U);
  for (const auto & point : cloud) {
    EXPECT_FLOAT_EQ(point.z, 0.5F);
  }
}

TEST_F(PGMapServerTest, EmptyGroundSafelyMergesFiniteSurface)
{
  writeSurfaceMergePoseGraph(false);
  auto server = makeServer(
    "empty_ground_surface_merge_map", 0.005,
    {
      rclcpp::Parameter("merge_non_ground_surface_into_mapcloud", true),
      rclcpp::Parameter("surface_ground_exclusion_radius", 0.05)
    });
  auto message = receiveLatchedCloud(server, "mapcloud");
  ASSERT_NE(message, nullptr);
  PointCloud cloud;
  pcl::fromROSMsg(*message, cloud);
  EXPECT_EQ(cloud.size(), 12U);
}

TEST_F(PGMapServerTest, RemovesRiserBoundedByTwoGroundTreads)
{
  std::filesystem::remove_all(test_dir_ / "pcd");
  std::filesystem::create_directories(test_dir_ / "pcd");
  pcl::PointCloud<PointTypePose> poses;
  PointTypePose pose{};
  poses.push_back(pose);
  ASSERT_EQ(
    pcl::io::savePCDFileBinary((test_dir_ / "poses.pcd").string(), poses),
    0);

  PointCloud ground;
  PointCloud surface;
  for (int i = 0; i < 12; ++i) {
    const float y = static_cast<float>(i) * 0.02F;
    for (const float tread_z : {0.0F, 0.18F}) {
      dddmr_pg_map_server::pcl_t tread{};
      tread.y = y;
      tread.z = tread_z;
      ground.push_back(tread);
    }
    for (const float riser_z : {0.03F, 0.06F, 0.09F, 0.12F, 0.15F}) {
      dddmr_pg_map_server::pcl_t riser{};
      riser.y = y;
      riser.z = riser_z;
      surface.push_back(riser);
    }
    dddmr_pg_map_server::pcl_t wall{};
    wall.x = 0.50F;
    wall.y = y;
    wall.z = 0.50F;
    surface.push_back(wall);
  }
  ASSERT_EQ(
    pcl::io::savePCDFileBinary(
      (test_dir_ / "pcd" / "0_ground.pcd").string(), ground),
    0);
  ASSERT_EQ(
    pcl::io::savePCDFileBinary(
      (test_dir_ / "pcd" / "0_surface.pcd").string(), surface),
    0);

  auto server = makeServer(
    "riser_exclusion_map", 0.005,
    {
      rclcpp::Parameter("merge_non_ground_surface_into_mapcloud", true),
      rclcpp::Parameter("surface_ground_exclusion_radius", 0.10)
    });
  auto message = receiveLatchedCloud(server, "mapcloud");
  ASSERT_NE(message, nullptr);
  PointCloud cloud;
  pcl::fromROSMsg(*message, cloud);
  ASSERT_EQ(cloud.size(), 12U);
  for (const auto & point : cloud) {
    EXPECT_FLOAT_EQ(point.x, 0.50F);
    EXPECT_FLOAT_EQ(point.z, 0.50F);
  }
}

TEST_F(PGMapServerTest, EmptyKeyFrameKeepsPoseIndexForAllCloudTypes)
{
  auto server = makeServer("indexed_map", 0.2);

  auto empty_response = requestKeyFrame(server, 1);
  EXPECT_EQ(empty_response->key_frame_cloud.width, 0U);
  EXPECT_EQ(empty_response->key_frame_cloud_base_link.width, 0U);
  EXPECT_EQ(empty_response->key_frame_surface.width, 0U);
  EXPECT_EQ(empty_response->key_frame_surface_base_link.width, 0U);
  EXPECT_EQ(empty_response->key_frame_ground.width, 0U);
  EXPECT_EQ(empty_response->key_frame_ground_base_link.width, 0U);

  auto final_response = requestKeyFrame(server, 2);
  PointCloud feature;
  PointCloud surface;
  PointCloud ground;
  pcl::fromROSMsg(final_response->key_frame_cloud, feature);
  pcl::fromROSMsg(final_response->key_frame_surface, surface);
  pcl::fromROSMsg(final_response->key_frame_ground, ground);
  ASSERT_EQ(feature.size(), 1U);
  ASSERT_EQ(surface.size(), 1U);
  ASSERT_EQ(ground.size(), 1U);
  EXPECT_FLOAT_EQ(feature.front().x, 21.0F);
  EXPECT_FLOAT_EQ(surface.front().x, 21.0F);
  EXPECT_FLOAT_EQ(ground.front().x, 21.0F);
}
}  // namespace
