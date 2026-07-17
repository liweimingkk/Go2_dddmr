#ifndef LEGO_LOAM_BOR__MOUTH_GROUND_SURFACE_H_
#define LEGO_LOAM_BOR__MOUTH_GROUND_SURFACE_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lego_loam_bor
{

  struct MouthGroundSurfaceConfig
  {
    double normal_radius {0.18};
    std::size_t minimum_neighbors {6};
    double maximum_slope {0.6108652381980153}; // 35 degrees
    // A deliberate angular dead band separates traversable surfaces from
    // verified obstacles.  Planes between the two limits stay unknown instead
    // of being forced into either mapping layer.
    double minimum_obstacle_slope {0.9599310885968813}; // 55 degrees
    double maximum_planarity_residual {0.035};
    double minimum_planar_spread {0.020};
    // The seed band describes the current support surface below base_link.  It
    // is deliberately not a stair-height window: once a seed is found, the
    // connected-surface search can walk through any number of height levels.
    double support_seed_z_min {-0.50};
    double support_seed_z_max {-0.18};
    double support_seed_x_max {0.70};
    double support_seed_y_abs {0.55};
    double support_reference_radius {0.12};
    double support_connection_radius {0.34};
    double maximum_step_height {0.26};
    double same_surface_height_tolerance {0.05};
    double minimum_patch_span {0.20};
    double minimum_patch_minor_span {0.10};
    std::size_t minimum_supported_component_points {8};
    std::size_t minimum_stair_height_levels {3};
  };

  bool validateMouthGroundSurfaceConfig(
    const MouthGroundSurfaceConfig & config, std::string * error = nullptr);

  struct MouthGroundSurfaceClassification
  {
    std::vector<std::uint8_t> supported_ground;
    std::vector<std::uint8_t> verified_obstacle;
  };

  // Classify into three states: supported_ground, verified_obstacle, or
  // unknown when both masks are zero.  In particular, an unsupported or
  // low-confidence horizontal return is unknown; it must not be written to
  // the obstacle map merely because it was not accepted as ground.
  MouthGroundSurfaceClassification classifyMouthGroundSurfaceDetailed(
    const pcl::PointCloud<pcl::PointXYZI> & gravity_aligned_cloud,
    const MouthGroundSurfaceConfig & config,
    const pcl::PointCloud<pcl::PointXYZI> * support_reference = nullptr);

// Input points must be expressed in a robot-local frame whose +Z axis is
// approximately gravity aligned.  The returned mask has one entry per input
// point.  A point is accepted only when it is locally planar, within the slope
// envelope, and connected to a current support-surface seed.  This keeps
// floating horizontal surfaces out of mapground while allowing multiple stair
// treads without encoding a particular stair height.
  std::vector < std::uint8_t > classifyMouthGroundSurface(
    const pcl::PointCloud < pcl::PointXYZI > &gravity_aligned_cloud,
    const MouthGroundSurfaceConfig & config,
    const pcl::PointCloud < pcl::PointXYZI > *support_reference = nullptr);

}  // namespace lego_loam_bor

#endif  // LEGO_LOAM_BOR__MOUTH_GROUND_SURFACE_H_
