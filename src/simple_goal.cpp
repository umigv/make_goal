#include "simple_goal.h"

#include "message_utils.h"

namespace umigv {
namespace make_goal {

ActionGoalT as_action_goal(const SimpleGoal &goal,
                           ActionGoalT pattern) noexcept {
    pattern.goal.target_pose.pose = goal.target_pose;

	return pattern;
}

std::ostream& operator<<(std::ostream &os, const SimpleGoal &goal) {
    return os << goal.target_pose;
}

std::istream& operator>>(std::istream &is, SimpleGoal &goal) {
    return is >> goal.target_pose;
}

} // namespace make_goal
} // namespace umigv
