#ifndef UMIGV_MAKE_GOAL_GOAL_DIRECTOR_H
#define UMIGV_MAKE_GOAL_GOAL_DIRECTOR_H

#include "simple_goal.h"

#include <iostream>
#include <stdexcept>
#include <vector>

#include <boost/optional.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <umigv_utilities/types.hpp>

namespace umigv {
namespace make_goal {

class GoalDirector;

class GoalDirectorBuilder {
public:
    GoalDirectorBuilder& with_node(ros::NodeHandle &node) noexcept;

    GoalDirectorBuilder& from_stream(std::istream &is) noexcept;

    GoalDirectorBuilder& with_pattern(ActionGoalT pattern) noexcept;

    GoalDirectorBuilder& with_threshold(f64 threshold) noexcept;

    // must be called on an rvalue
    // throws std::logic_error if not fully initialized
    GoalDirector build();

private:
    ros::NodeHandle *node_ = nullptr;
    std::istream *is_ = nullptr;
    boost::optional<ActionGoalT> pattern_ = boost::none;
    boost::optional<f64> threshold_ = boost::none;
};

class GoalDirector {
public:
    friend GoalDirectorBuilder;

    void update_odometry(const nav_msgs::Odometry::ConstPtr &odom_ptr);

    void publish_goal(const ros::TimerEvent&);

private:
    GoalDirector(ros::Publisher publisher, std::vector<SimpleGoal> goals,
				 ActionGoalT pattern, f64 threshold) noexcept;

    bool is_current_valid() const noexcept;

    ros::Publisher publisher_;
    std::vector<SimpleGoal> goals_;
    std::vector<SimpleGoal>::const_iterator current_iter_ = goals_.cbegin();
    ActionGoalT pattern_;
    f64 distance_threshold_;
};

} // namespace make_goal
} // namespace umigv

#endif
