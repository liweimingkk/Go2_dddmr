#ifndef DDDMR_SYS_CORE__ACTION_CLIENT_COMPAT_HPP_
#define DDDMR_SYS_CORE__ACTION_CLIENT_COMPAT_HPP_

#include <future>

namespace dddmr_sys_core
{
namespace action_client_compat
{

// Foxy delivers an action goal response as a shared_future, while newer ROS 2
// releases deliver the goal handle directly.
template<typename GoalHandleT>
typename GoalHandleT::SharedPtr unwrap_goal_response(
  const typename GoalHandleT::SharedPtr & response)
{
  return response;
}

template<typename GoalHandleT>
typename GoalHandleT::SharedPtr unwrap_goal_response(
  const std::shared_future<typename GoalHandleT::SharedPtr> & response)
{
  return response.get();
}

}  // namespace action_client_compat
}  // namespace dddmr_sys_core

#endif  // DDDMR_SYS_CORE__ACTION_CLIENT_COMPAT_HPP_
