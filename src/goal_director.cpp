#include "goal_director.h"

#include "message_utils.h"

#include <iterator>
#include <utility>

#include <ros/time.h>

namespace umigv {
namespace make_goal {

GoalDirectorBuilder&
GoalDirectorBuilder::with_node(ros::NodeHandle &node) noexcept {
    node_ = &node;

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::from_stream(std::istream &is) noexcept {
    is_ = &is;

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::with_pattern(ActionGoalT pattern) noexcept {
    pattern_ = std::move(pattern);

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::with_threshold(f64 threshold) noexcept {
    threshold_ = threshold;

    return *this;
}

GoalDirector GoalDirectorBuilder::build() {
    if (!node_ || !is_ || !pattern_ || !threshold_) {
        throw std::logic_error{ "GoalDirectorBuilder::build" };
    }

    return { node_->advertise<ActionGoalT>("move_base/goal", 10),
             std::vector<SimpleGoal>(std::istream_iterator<SimpleGoal>{ *is_ },
                                     std::istream_iterator<SimpleGoal>{ }),
             std::move(pattern_.value()),
             threshold_.value() };
}

void GoalDirector::update_odometry(
    const nav_msgs::Odometry::ConstPtr &odom_ptr
) {
    if (!is_current_valid()) {
        return;
    }

    const geometry_msgs::Pose &pose = odom_ptr->pose.pose;

    const f64 pose_distance =
        distance(pose.position, current_iter_->target_pose.position);

    if (pose_distance < distance_threshold_) {
        ++current_iter_;
    }
}

void GoalDirector::publish_goal(const ros::TimerEvent&) {
    if (!is_current_valid()) {
        return;
    }

    const auto now = ros::Time::now();

    pattern_.header.stamp = now;
    pattern_.goal.target_pose.header.stamp = now;
    publisher_.publish(as_action_goal(*current_iter_, pattern_));
    ++pattern_.header.seq;
    ++pattern_.goal.target_pose.header.seq;

    ROS_INFO_STREAM("published goal "
                    << std::distance(goals_.cbegin(), current_iter_));
}

GoalDirector::GoalDirector(ros::Publisher publisher,
                           std::vector<SimpleGoal> goals,
                           ActionGoalT pattern, const f64 threshold) noexcept
: publisher_{ std::move(publisher) }, goals_{ std::move(goals) },
  pattern_{ std::move(pattern) }, distance_threshold_{ threshold } { }

bool GoalDirector::is_current_valid() const noexcept {
    return current_iter_ >= goals_.cbegin() && current_iter_ < goals_.cend();
}

} // namespace make_goal
} // namespace umigv
