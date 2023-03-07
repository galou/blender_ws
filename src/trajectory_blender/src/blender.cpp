#include <trajectory_blender/blender.hpp>

#include <trajectory_blender/plan_conditions.hpp>  // PlanConditions.

#include <Eigen/Geometry>  // Eigen::{Isometry3d,Vector3d,VectorXd,...}.

#include <moveit/moveit_cpp/planning_component.h>  // moveit_cpp::PlanningComponent.
#include <moveit/planning_scene/planning_scene.h>  // planning_scene::PlanningSceneConstPtr.
#include <moveit/robot_state/robot_state.h>  // moveit::core::RobotState.
#include <moveit/robot_trajectory/robot_trajectory.h>  // robot_trajectory::{RobotTrajectory,RobotTrajectoryPtr}.

#include <moveit_msgs/msg/move_it_error_codes.hpp>  // moveit_msgs::msg::MoveItErrorCodes.
#include <moveit_msgs/msg/robot_state.hpp>  // moveit_msgs::msg::RobotState.

#include <moveit/trajectory_processing/ruckig_traj_generator.h>  // trajectory_processing::RuckigGenerator.

#include <rclcpp/logging.hpp>  // rclcpp::Logger.

#include <algorithm>  // std::{copy_n,find,max,min,partial_sort,transform}.
#include <cstdint>  // std::size_t.
#include <exception>  /// std::runtime_error.
#include <limits>  // std::numeric_limits.
#include <numeric>  // For std::iota.
#include <string>  // std::string.
#include <vector>  // std::vector.

namespace
{

/* Constant to check for equality of values. */
static constexpr double EPSILON = 1e-4;

bool
isRobotStateEqual(
    const moveit::core::RobotState& state1,
    const std::string& joint_group_name1,
    const moveit::core::RobotState& state2,
    const std::string& joint_group_name2,
    double epsilon)
{
  Eigen::VectorXd joint_position_1;
  Eigen::VectorXd joint_position_2;

  state1.copyJointGroupPositions(joint_group_name1, joint_position_1);
  state2.copyJointGroupPositions(joint_group_name2, joint_position_2);

  if ((joint_position_1 - joint_position_2).norm() > epsilon)
  {
    return false;
  }

  Eigen::VectorXd joint_velocity_1;
  Eigen::VectorXd joint_velocity_2;

  state1.copyJointGroupVelocities(joint_group_name1, joint_velocity_1);
  state2.copyJointGroupVelocities(joint_group_name2, joint_velocity_2);

  if ((joint_velocity_1 - joint_velocity_2).norm() > epsilon)
  {
    return false;
  }

  return true;
}

double
median(const std::vector<double> & values)
{
  const bool is_even = not(values.size() % 2);
  const std::size_t n = values.size() / 2;

  std::vector<std::size_t> vi(values.size());
  std::iota(vi.begin(), vi.end(), 0);   /* 0, 1, 2, ... */

  std::partial_sort(vi.begin(), vi.begin() + n + 1, vi.end(),
      [&](std::size_t lhs, std::size_t rhs) {return values[lhs] < values[rhs];});

  if (is_even)
  {
    return 0.5 * (values[vi.at(n-1)] + values[vi.at(n)]);
  }
  else
  {
    return values[vi.at(n)];
  }
}

double
determineSamplingTime(
    const robot_trajectory::RobotTrajectoryPtr& first_trajectory,
    const robot_trajectory::RobotTrajectoryPtr& second_trajectory,
    rclcpp::Logger & logger)
{
  // The last sample is ignored because it is allowed to violate the sampling
  // time.
  std::size_t n1 = first_trajectory->getWayPointCount() - 1;
  std::size_t n2 = second_trajectory->getWayPointCount() - 1;
  if ((n1 < 1) and (n2 < 1))
  {
    RCLCPP_ERROR_STREAM(logger, "Both trajectories do not have enough points to determine sampling time.");
    return false;
  }

  if (n2 >= 1)
  {
    /* Prefer to take the sampling time from the second trajectory because it
     * has less chances to be a blended trajectory with irregular sampling time. */
    std::vector<double> dt(n2);
    std::transform(second_trajectory->begin(), std::next(second_trajectory->begin(), n2), dt.begin(),
        [] (const auto & it) {return it.second;});
    return median(dt);
  }
  else
  {
    std::vector<double> dt(n1);
    std::transform(first_trajectory->begin(), std::next(first_trajectory->begin(), n1), dt.begin(),
        [] (const auto & it) {return it.second;});
    return median(dt);
  }
}

bool
isRobotStateWithoutVelocity(const moveit::core::RobotState& state,
    const std::string& group, double epsilon,
    rclcpp::Logger & logger)
{
  Eigen::VectorXd joint_variable;
  state.copyJointGroupVelocities(group, joint_variable);
  if (joint_variable.norm() > epsilon)
  {
    RCLCPP_DEBUG(logger, "Joint velocities are not zero.");
    return false;
  }
  return true;
}

bool
validateRequest(
    const robot_trajectory::RobotTrajectoryPtr & traj1,
    const std::string & link_name1,
    const robot_trajectory::RobotTrajectoryPtr & traj2,
    const std::string & link_name2,
    double blend_radius,
    double & sampling_time,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    rclcpp::Logger & logger)
{
  RCLCPP_DEBUG(logger, "Validating the trajectory blend request.");

  /* Check that links exist. */
  if ((not traj1->getRobotModel()->hasLinkModel(link_name1)) and
      (not traj1->getLastWayPoint().hasAttachedBody(link_name1)))
  {
    RCLCPP_ERROR_STREAM(logger, "Unknown link name: " << link_name1);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
    return false;
  }
  if ((not traj2->getRobotModel()->hasLinkModel(link_name2)) and
      (not traj2->getLastWayPoint().hasAttachedBody(link_name2)))
  {
    RCLCPP_ERROR_STREAM(logger, "Unknown link name: " << link_name2);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
    return false;
  }

  if (blend_radius <= 0.0)
  {
    RCLCPP_ERROR(logger, "Blending radius must be positive");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // end position of the first trajectory and start position of second
  // trajectory must be the same
  if (not isRobotStateEqual(
        traj1->getLastWayPoint(), traj1->getGroupName(), traj2->getFirstWayPoint(), traj1->getGroupName(),
        EPSILON))
  {
    RCLCPP_ERROR_STREAM(
        logger, "During blending the last point of the preceding and the first point of the succeeding trajectory");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  sampling_time = determineSamplingTime(traj1, traj2, logger);

  // end position of the first trajectory and start position of second
  // trajectory must have zero velocities/accelerations
  if ((not isRobotStateWithoutVelocity(traj1->getLastWayPoint(), traj1->getGroupName(),
        EPSILON, logger))
      or
      (not isRobotStateWithoutVelocity(traj2->getFirstWayPoint(), traj1->getGroupName(),
        EPSILON, logger)))
  {
    RCLCPP_ERROR(logger, "Intersection point of the blending trajectories has non-zero velocities.");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  return true;
}

bool
intersectionFound(
    const Eigen::Vector3d & p_center,
    const Eigen::Vector3d & p_current,
    const Eigen::Vector3d & p_next,
    double r)
{
  return ((p_current - p_center).norm() <= r) && ((p_next - p_center).norm() >= r);
}

bool
linearSearchIntersectionPoint(
    const std::string & link_name,
    const Eigen::Vector3d & center_position,
    double r,
    const robot_trajectory::RobotTrajectoryPtr& traj,
    bool inverse_order,
    std::size_t & index,
    rclcpp::Logger & logger)
{
  RCLCPP_DEBUG(logger, "Start linear search for intersection point.");

  const std::size_t waypoint_num = traj->getWayPointCount();

  if (inverse_order)
  {
    for (std::size_t i = waypoint_num - 1; i > 0; --i)
    {
      // TODO: optimization: avoid calling getFrameTransform() twice.
      if (intersectionFound(center_position, traj->getWayPointPtr(i)->getFrameTransform(link_name).translation(),
                            traj->getWayPointPtr(i - 1)->getFrameTransform(link_name).translation(), r))
      {
        index = i;
        return true;
      }
    }
  }
  else
  {
    for (std::size_t i = 0; i < waypoint_num - 1; ++i)
    {
      // TODO: optimization: avoid calling getFrameTransform() twice.
      if (intersectionFound(center_position, traj->getWayPointPtr(i)->getFrameTransform(link_name).translation(),
                            traj->getWayPointPtr(i + 1)->getFrameTransform(link_name).translation(), r))
      {
        index = i;
        return true;
      }
    }
  }

  return false;
}

bool
searchIntersectionPoints(
    const robot_trajectory::RobotTrajectoryPtr & traj1,
    const std::string & link_name1,
    const robot_trajectory::RobotTrajectoryPtr & traj2,
    const std::string & link_name2,
    double blend_radius,
    std::size_t & first_intersection_index,
    std::size_t & second_intersection_index,
    rclcpp::Logger & logger)
{
  RCLCPP_DEBUG(logger, "Search for start and end point of blending trajectory.");

  // Position of the center of the blend sphere
  // (i.e. last point of the first trajectory and first point of the second trajectory).
  Eigen::Isometry3d corner = traj1->getLastWayPoint().getFrameTransform(link_name1);

  // Search for intersection points according to distance.
  if (not linearSearchIntersectionPoint(
          link_name1, corner.translation(), blend_radius, traj1, true, first_intersection_index, logger))
  {
    RCLCPP_DEBUG_STREAM(logger, "Intersection point of first trajectory not found.");
    return false;
  }
  RCLCPP_DEBUG_STREAM(logger, "Intersection point of first trajectory found at index " << first_intersection_index
      << ":\n" << traj1->getWayPointPtr(first_intersection_index)->getFrameTransform(link_name1).translation());

  if (not linearSearchIntersectionPoint(
          link_name2, corner.translation(), blend_radius, traj2, false, second_intersection_index, logger))
  {
    RCLCPP_DEBUG_STREAM(logger, "Intersection point of second trajectory not found.");
    return false;
  }
  RCLCPP_DEBUG_STREAM(logger, "Intersection point of second trajectory found at index " << second_intersection_index
      << ":\n" << traj2->getWayPointPtr(second_intersection_index)->getFrameTransform(link_name2).translation());

  return true;
}

trajectory_blender::PlanConditions
mergePlanConditions(
    const trajectory_blender::PlanConditions & cond1,
    const trajectory_blender::PlanConditions & cond2)
{
  trajectory_blender::PlanConditions out_cond;
  out_cond.parameters.max_velocity_scaling_factor = std::min(
      cond1.parameters.max_velocity_scaling_factor,
      cond2.parameters.max_velocity_scaling_factor);
  out_cond.parameters.max_acceleration_scaling_factor = std::min(
      cond1.parameters.max_acceleration_scaling_factor,
      cond2.parameters.max_acceleration_scaling_factor);
  return out_cond;
}

double
distToCorner(
    robot_trajectory::RobotTrajectory & traj,
    const std::string & link_name,
    const Eigen::Vector3d & corner)
{
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & [wp, _] : traj)
  {
    const double this_dist = (wp->getFrameTransform(link_name).translation() - corner).norm();
    if (this_dist < min_dist)
    {
      min_dist = this_dist;
    }
  }

  return min_dist;
}

bool
generateBlendJointTrajectory(
    const robot_trajectory::RobotTrajectoryPtr & traj1,
    const std::string & link_name1,
    const trajectory_blender::PlanConditions & cond1,
    const robot_trajectory::RobotTrajectoryPtr & traj2,
    const std::string & link_name2,
    const trajectory_blender::PlanConditions & cond2,
    double blend_radius,
    const planning_scene::PlanningSceneConstPtr & /* scene */,
    robot_trajectory::RobotTrajectory & blend_trajectory,
    std::size_t & blend_start,
    std::size_t & blend_end,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    rclcpp::Logger & logger)
{
  if (traj1->getGroup() != traj2->getGroup())
  {
    RCLCPP_ERROR_STREAM(logger, "Trajectories are for different groups");
    return false;
  }

  Eigen::Isometry3d corner =
      traj1->getLastWayPointPtr()->getFrameTransform(link_name1);

  const auto blend_cond = mergePlanConditions(cond1, cond2);

  /* Radius of the sphere where the final blending will start and end.
   * `blend_radius` is the distance from the corner to the final trajectory and
   * is smaller. */
  std::size_t first_intersection_index;
  std::size_t second_intersection_index;
  if (not searchIntersectionPoints(traj1, link_name1, traj2, link_name2,
        blend_radius, first_intersection_index, second_intersection_index, logger))
  {
    RCLCPP_ERROR_STREAM(logger, "generateBlendJointTrajectory(), radius " << blend_radius << " too large");
    return false;
  }

  /* Generate a two-point trajectory as input for generate(). */
  auto blend_traj = robot_trajectory::RobotTrajectory{traj1->getRobotModel(), traj1->getGroup()};
  const double timestep = std::min(traj1->getAverageSegmentDuration(), traj2->getAverageSegmentDuration());
  /* Irrelevant dt parameter. */
  blend_traj.addPrefixWayPoint(*traj1->getWayPointPtr(first_intersection_index), 0.0);
  blend_traj.addSuffixWayPoint(*traj2->getWayPointPtr(second_intersection_index), 1.0);  // Must be > 0?
  const bool timing_success = trajectory_processing::RuckigGenerator::generate(blend_traj, timestep,
      blend_cond.parameters.max_velocity_scaling_factor, blend_cond.parameters.max_acceleration_scaling_factor);
  if (not timing_success)
  {
    RCLCPP_ERROR(logger, "Cannot generate the blend trajectory");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    blend_start = traj1->getWayPointCount() - 1;
    blend_end = 0;
    blend_trajectory.clear();
    return false;
  }

  /* Check that the generated trajectory is inside the radius. */
  const double this_dist = distToCorner(blend_traj, link_name1, corner.translation());

  if (this_dist > blend_radius)
  {
    RCLCPP_ERROR_STREAM(logger, "generateBlendJointTrajectory(), generated trajectory too far from original, "
        << this_dist << " vs. " << blend_radius);
  }

  blend_start = first_intersection_index;
  blend_end = second_intersection_index;
  blend_trajectory = std::move(blend_traj);
  return true;
}

moveit_msgs::msg::RobotState
mergeWithCurrentState(
    const std::vector<std::string> & names, const std::vector<double> & positions,
    const planning_scene::PlanningSceneConstPtr & planning_scene,
    rclcpp::Logger & logger)
{
  if (names.size() != positions.size())
  {
    RCLCPP_FATAL_STREAM(logger, "mergeWithCurrentState(): names and positions have incompatible number of values, " << names.size() << " vs. " << positions.size());
    throw std::runtime_error("mergeWithCurrentState(): names and positions have incompatible number of values");
  }
  const moveit::core::RobotState state{planning_scene->getCurrentState()};
  moveit_msgs::msg::RobotState out_state;
  out_state.joint_state.name = state.getVariableNames();
  out_state.joint_state.position.resize(state.getVariableCount());
  std::copy_n(state.getVariablePositions(), state.getVariableCount(), out_state.joint_state.position.begin());
  for (std::size_t j = 0; j < names.size(); ++j)
  {
    const auto & this_name = names[j];
    const auto & this_position = positions[j];
    const auto name_it = std::find(out_state.joint_state.name.cbegin(), out_state.joint_state.name.cend(), this_name);
    if (name_it == out_state.joint_state.name.cend())
    {
      RCLCPP_FATAL_STREAM(logger, "mergeWithCurrentState(): joint name '" << this_name << "' not found in robot state");
      std::runtime_error("mergeWithCurrentState(): joint name not found in robot state");
    }
    const auto this_index = std::distance(out_state.joint_state.name.cbegin(), name_it);
    out_state.joint_state.position[this_index] = this_position;
  }
  return out_state;
}

} /* Anonymous namespace. */

namespace trajectory_blender
{

using PlanSolution = moveit_cpp::PlanningComponent::PlanSolution;

Blender::Blender()
{
}

Blender::~Blender()
{
}

std::vector<PlanSolution>
Blender::blend(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const std::vector<robot_trajectory::RobotTrajectoryPtr>& trajectories,
    const std::vector<double>& radii,
    const std::vector<std::string>& link_names,
    const std::vector<PlanConditions>& plan_conditions,
    rclcpp::Logger & logger)
{
  RCLCPP_DEBUG(logger, "Start trajectory blending using joint interpolation.");

  std::vector<PlanSolution> solutions;

  if (trajectories.size() != radii.size())
  {
    RCLCPP_FATAL_STREAM(logger, "Trajectory count (" << trajectories.size() << ") and radius count (" << radii.size() << ") are not compatible");
    return solutions;
  }

  if (trajectories.size() != link_names.size())
  {
    RCLCPP_FATAL_STREAM(logger, "Trajectory count (" << trajectories.size() << ") and link count (" << link_names.size() << ") are not compatible");
    return solutions;
  }

  if (trajectories.size() != plan_conditions.size())
  {
    RCLCPP_FATAL_STREAM(logger, "Trajectory count (" << trajectories.size() << ") and condition count (" << plan_conditions.size() << ") are not compatible");
    return solutions;
  }

  auto trajectory_to_continue = robot_trajectory::RobotTrajectoryPtr{};
  std::string const * last_link_name_ptr;
  PlanConditions const * last_plan_conditions_ptr;
  double last_radius;

  for (std::size_t i = 0; i < trajectories.size(); ++i)
  {
    if (trajectory_to_continue == nullptr)
    {
      /* Start of a new trajectory. */
      trajectory_to_continue = std::make_shared<robot_trajectory::RobotTrajectory>(trajectories[i]->getRobotModel(), trajectories[i]->getGroup());
      trajectory_to_continue->append(*trajectories[i], 0.0);
      last_link_name_ptr = &link_names[i];
      last_plan_conditions_ptr = &plan_conditions[i];
      last_radius = radii[i];
      continue;
    }

    if (trajectory_to_continue->getGroup() != trajectories[i]->getGroup())
    {
      /* New PlanSolution. */
      if (not trajectory_to_continue->empty())
      {
        PlanSolution solution;
        const auto & wp = trajectory_to_continue->getFirstWayPointPtr();
        std::vector<double> start_positions(wp->getVariableCount());
        std::copy_n(wp->getVariablePositions(), wp->getVariableCount(), start_positions.begin());
        solution.start_state = mergeWithCurrentState(
            wp->getVariableNames(),
            start_positions,
            planning_scene, logger);

        solution.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory_to_continue->getRobotModel(),
            trajectory_to_continue->getGroup());
        solution.trajectory->append(*trajectory_to_continue, 0.0);
        solution.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        solutions.emplace_back(solution);
      }

      /* Reset `trajectory_to_continue` for next iteration. */
      RCLCPP_DEBUG_STREAM(logger, "Reset trajectory_to_continue at trajectory " << i); // DEBUG
      trajectory_to_continue.reset();
      continue;
    }

    if (last_radius == 0.0)
    {
      /* No blending, just add the trajectory without the first duplicated point
       * at the end of `trajectory_to_continue`. */
      trajectory_to_continue->append(*trajectories[i], 0.0, 1);
      last_link_name_ptr = &link_names[i];
      last_plan_conditions_ptr = &plan_conditions[i];
      last_radius = radii[i];
      continue;
    }

    double sampling_time;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    if (!validateRequest(
          trajectory_to_continue, *last_link_name_ptr,
          trajectories[i], link_names[i],
          last_radius,
          sampling_time, error_code, logger))
    {
      RCLCPP_ERROR(logger, "Trajectory blend request is not valid.");
      return std::vector<PlanSolution>{};
    }

    auto blend_trajectory = robot_trajectory::RobotTrajectory{trajectory_to_continue->getRobotModel(),
                                                              trajectory_to_continue->getGroup()};
    std::size_t first_intersection_index;
    std::size_t second_intersection_index;
    if (not generateBlendJointTrajectory(
          trajectory_to_continue, *last_link_name_ptr, *last_plan_conditions_ptr,
          trajectories[i], link_names[i], plan_conditions[i],
          last_radius,
          planning_scene,
          blend_trajectory, first_intersection_index, second_intersection_index, error_code, logger))
    {
      RCLCPP_INFO(logger, "Failed to generate joint trajectory for blending trajectory.");
      return solutions;
    }

    /* Future `trajectory_to_continue`, with some points from
     * `trajectory_to_continue`, all points from `blend_trajectory`, and some
     * points from `trajectories[i]`. */
    auto new_trajectory = robot_trajectory::RobotTrajectory{trajectory_to_continue->getRobotModel(),
                                                            trajectory_to_continue->getGroup()};

    /* Add the points from `trajectory_to_continue` without the point at
     * `first_intersection_index`. */
    new_trajectory.append(*trajectory_to_continue, 0.0, 0, first_intersection_index);

    /* Add the points from the blending trajectory without the last one
     * that is duplicated. */
    const std::size_t stop = static_cast<std::size_t>(std::max(static_cast<int>(blend_trajectory.getWayPointCount()) - 1,
          static_cast<int>(0)));
    new_trajectory.append(blend_trajectory, sampling_time, 0, stop);

    /* Add the points from the second trajectory. */
    new_trajectory.append(*trajectories[i], 0.0, second_intersection_index);

    /* Prepare for next iteration. */
    trajectory_to_continue->swap(new_trajectory);
    last_link_name_ptr = &link_names[i];
    last_plan_conditions_ptr = &plan_conditions[i];
    last_radius = radii[i];

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  /* Add the last, now finished, `trajectory_to_continue`. */
  if ((trajectory_to_continue != nullptr) and (not trajectory_to_continue->empty()))
  {
    PlanSolution solution;
    const auto & wp = trajectory_to_continue->getFirstWayPointPtr();
    std::vector<double> start_positions(wp->getVariableCount());
    std::copy_n(wp->getVariablePositions(), wp->getVariableCount(), start_positions.begin());
    solution.start_state = mergeWithCurrentState(
        wp->getVariableNames(),
        start_positions,
        planning_scene, logger);

    solution.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(trajectory_to_continue->getRobotModel(), trajectory_to_continue->getGroup());
    solution.trajectory->append(*trajectory_to_continue, 0.0);
    solution.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    solutions.emplace_back(solution);
  }

  return solutions;
}

}  // namespace trajectory_blender
