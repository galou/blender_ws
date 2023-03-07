#pragma once

#include <trajectory_blender/plan_conditions.hpp>  // PlanConditions.
#include <trajectory_blender/visibility_control.h>

#include <moveit/moveit_cpp/planning_component.h>  // moveit_cpp::PlanningComponent.
#include <moveit/planning_scene/planning_scene.h>  // planning_scene::PlanningSceneConstPtr.
#include <moveit/robot_trajectory/robot_trajectory.h>  // robot_trajectory::RobotTrajectoryPtr.

#include <rclcpp/logging.hpp>  // rclcpp::Logger.

#include <vector>  // std::vector.

namespace trajectory_blender
{

class Blender
{
public:

  using PlanSolution = moveit_cpp::PlanningComponent::PlanSolution;

  Blender();

  virtual ~Blender();

  /** Join two or more trajectories by changing points inside a given radius
   *
   * A new ouput trajectory is added to the output list when the group of the
   * currently-processed input trajectory is different than the one of the
   * previous trajectory.
   *
   * @param[in] planning_scene Planning scene used to get the current robot state
   *                           for joints that are not in the input trajectories
   * @param[in] trajectories C0 continuous trajectories with null velocity at
   *                         start and end of each trajectory.
   * @param[in] radii Radius of the sphere to determine which points of the
   *                  original trajectories will be removed.
   *                  The generated blending trajectories will be strictly inside
   *                  this sphere.
   *                  It must have the same number of values than the number of
   *                  input trajectories, the last radius value being unused.
   * @param[in] link_names Names of the links the trajectories were planned for.
   *                  Must have the same size as `trajectories`.
   * @param[in] plan_conditions Conditions the trajectories were planned with.
   *                  The maximum velocity and acceleration of the blended
   *                  trajectory is obtained from them.
   *                  Must have the same size as `trajectories`.
   * @param[in] logger Used to send ROS logs.
   */
  static std::vector<PlanSolution>
    blend(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const std::vector<robot_trajectory::RobotTrajectoryPtr>& trajectories,
        const std::vector<double>& radii,
        const std::vector<std::string>& link_names,
        const std::vector<PlanConditions>& plan_conditions,
        rclcpp::Logger & logger);
};

}  // namespace trajectory_blender

