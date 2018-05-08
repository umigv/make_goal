#ifndef UMIGV_MAKE_GOAL_SIMPLE_GOAL_H
#define UMIGV_MAKE_GOAL_SIMPLE_GOAL_H

#include <iostream>

#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace umigv {
namespace make_goal {

using ActionGoalT = move_base_msgs::MoveBaseActionGoal;

struct SimpleGoal {
    geometry_msgs::Pose target_pose;
};

ActionGoalT as_action_goal(const SimpleGoal &goal,
                           ActionGoalT pattern) noexcept;

std::ostream& operator<<(std::ostream &os, const SimpleGoal &goal);

std::istream& operator>>(std::istream &is, SimpleGoal &goal);

} // namespace make_goal
} // namespace umigv

#endif
