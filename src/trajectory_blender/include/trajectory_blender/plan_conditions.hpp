/** Structure that holds planning conditions.
 */

#pragma once

#include <optional>  // std::optional.
#include <sstream>  // std::ostringstream.
#include <string>  // std::string.
#include <string_view>  // std::string_view.
#include <tuple>  // std::tie.

#include <moveit/moveit_cpp/planning_component.h>  // moveit_cpp::PlanningComponent.

#include <moveit_msgs/msg/position_constraint.hpp>  // moveit_msgs::msg::PositionConstraint.

namespace trajectory_blender
{

struct PlanConditions
{
  // Implementation note: PlanConditions must have a default constructor to be
  // usable as value in std::map.
  PlanConditions(double vel = 0.0, double acc = 0.0)
  {
    parameters.max_velocity_scaling_factor = vel;
    parameters.max_acceleration_scaling_factor = acc;

    /* The default planning pipeline must be specified. But the planner
     * (`planner_id`) will be chosen according to OMPL configuration via
     * parameters (by group, etc.) */
    parameters.planning_pipeline = "ompl";
  }

  PlanConditions & setVel(double vel)
  {
    parameters.max_velocity_scaling_factor = vel;
    return *this;
  }

  PlanConditions & setAcc(double acc)
  {
    parameters.max_acceleration_scaling_factor = acc;
    return *this;
  }

  PlanConditions & setAttempts(unsigned int planning_attempts)
  {
    parameters.planning_attempts = planning_attempts;
    return *this;
  }

  PlanConditions & setPlanner(const std::string & pipeline, const std::string & planner)
  {
    parameters.planning_pipeline = pipeline;
    parameters.planner_id = planner;
    return *this;
  }

  PlanConditions & setLabel(const std::string_view & label)
  {
    this->label = label;
    return *this;
  }

  PlanConditions withVel(double vel) const
  {
    PlanConditions out_plan{*this};
    out_plan.setVel(vel);
    return out_plan;
  }

  PlanConditions withAcc(double acc) const
  {
    PlanConditions out_plan{*this};
    out_plan.setAcc(acc);
    return out_plan;
  }

  PlanConditions withAttempts(unsigned int planning_attempts) const
  {
    PlanConditions out_plan{*this};
    out_plan.setAttempts(planning_attempts);
    return out_plan;
  }

  PlanConditions withPlanner(const std::string & pipeline, const std::string & planner) const
  {
    PlanConditions out_plan{*this};
    out_plan.setPlanner(pipeline, planner);
    return out_plan;
  }

  PlanConditions withLabel(const std::string_view & label) const
  {
    PlanConditions out_plan{*this};
    out_plan.setLabel(label);
    return out_plan;
  }

  PlanConditions withLIN() const
  {
    PlanConditions out_plan{*this};
    out_plan.setPlanner("pilz", "LIN");
    out_plan.setAttempts(1);
    return out_plan;
  }

  PlanConditions withPTP() const
  {
    PlanConditions out_plan{*this};
    out_plan.setPlanner("pilz", "PTP");
    out_plan.setAttempts(1);
    return out_plan;
  }

  PlanConditions withLINI() const
  {
    PlanConditions out_plan{*this};
    out_plan.setPlanner("pilz", "LINI");
    out_plan.setAttempts(1);
    return out_plan;
  }

  bool operator==(const PlanConditions & rhs) const
  {
    return (std::tie(
          parameters.planning_pipeline,
          parameters.planner_id) ==
        std::tie(
          rhs.parameters.planning_pipeline,
          rhs.parameters.planner_id)
        )
      and (std::abs(parameters.max_velocity_scaling_factor - rhs.parameters.max_velocity_scaling_factor) < VELOCITY_EQUALITY_TOLERANCE)
      and (std::abs(parameters.max_acceleration_scaling_factor - rhs.parameters.max_acceleration_scaling_factor) < ACCELERATION_EQUALITY_TOLERANCE);
  }

  bool operator!=(const PlanConditions & rhs) const
  {
    return not (*this == rhs);
  }

  static constexpr double POSITION_EQUALITY_TOLERANCE = 0.0001;
  static constexpr double VELOCITY_EQUALITY_TOLERANCE = 0.001;
  static constexpr double ACCELERATION_EQUALITY_TOLERANCE = 0.01;

  moveit_cpp::PlanningComponent::PlanRequestParameters parameters;

  std::string label;
};

} /* namespace trajectory_blender */

// Implementation note: operators for stream output must be in global namespace.
inline
std::ostream &
operator<<(std::ostream & output_stream, const trajectory_blender::PlanConditions & pc)
{
  std::ostringstream label_oss;
  if (not pc.label.empty())
  {
    label_oss << " (LABEL " << pc.label << ")";
  }

  return output_stream
    << "["
    << pc.parameters.planning_pipeline << "/" << (pc.parameters.planner_id.empty() ? "?" : pc.parameters.planner_id)
    << ", vel " << pc.parameters.max_velocity_scaling_factor
    << ", acc " << pc.parameters.max_acceleration_scaling_factor
    << "]"
    << label_oss.str();
}
