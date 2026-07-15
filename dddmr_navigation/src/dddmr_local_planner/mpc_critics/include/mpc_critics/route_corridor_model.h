#ifndef MPC_CRITICS_ROUTE_CORRIDOR_MODEL_H_
#define MPC_CRITICS_ROUTE_CORRIDOR_MODEL_H_

#include <mpc_critics/scoring_model.h>

namespace mpc_critics
{

class RouteCorridorModel : public ScoringModel
{
public:
  RouteCorridorModel() = default;
  double scoreTrajectory(base_trajectory::Trajectory & trajectory) override;

protected:
  void onInitialize() override;

private:
  double max_xy_distance_{0.60};
  double max_z_distance_{0.35};
};

}  // namespace mpc_critics

#endif  // MPC_CRITICS_ROUTE_CORRIDOR_MODEL_H_
