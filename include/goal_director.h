#ifndef UMIGV_MAKE_GOAL_GOAL_DIRECTOR_H
#define UMIGV_MAKE_GOAL_GOAL_DIRECTOR_H

#include "message_utils.h"

#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <boost/optional.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <umigv_utilities/types.hpp>

namespace umigv {
namespace make_goal {

using GoalT = move_base_msgs::MoveBaseActionGoal;

class GoalDirector;

class GoalDirectorBuilder {
public:
    GoalDirectorBuilder& with_node(ros::NodeHandle &node) noexcept;

    GoalDirectorBuilder& from_stream(std::istream &is) noexcept;

    GoalDirectorBuilder& with_threshold(f64 threshold) noexcept;

    GoalDirectorBuilder& with_goal_frame(std::string frame) noexcept;

    GoalDirectorBuilder& with_goal_id(std::string id) noexcept;

    // must be called on an rvalue
    // throws std::logic_error if not fully initialized
    GoalDirector build();

private:
    ros::NodeHandle *node_ = nullptr;
    boost::optional<std::vector<GoalT>> goals_ = boost::none;
    boost::optional<f64> threshold_ = boost::none;
    boost::optional<std::string> frame_ = boost::none;
    boost::optional<std::string> id_ = boost::none;
};

class GoalDirector {
public:
    friend GoalDirectorBuilder;

    GoalDirector(GoalDirector &&other) noexcept;

    tf2::BufferCore& buffer() noexcept;

    void update_odometry(const nav_msgs::Odometry::ConstPtr &odom_ptr);

    void publish_goal(const ros::TimerEvent&);

private:
    GoalDirector(ros::Publisher publisher, std::vector<GoalT> goals,
                 f64 threshold) noexcept;

    bool is_goal_reached(const nav_msgs::Odometry &odom) const noexcept;

    boost::optional<std::reference_wrapper<const GoalT>>
    current() const noexcept;

    boost::optional<nav_msgs::Odometry>
    get_transformed_odom(const nav_msgs::Odometry &odom) const noexcept;

    boost::optional<std::size_t> current_index() const noexcept;

    ros::Publisher publisher_;
    std::vector<GoalT> goals_;
    f64 distance_threshold_;
    std::vector<GoalT>::const_iterator current_iter_ = goals_.cbegin();
    std::size_t seq_ = 0;
    tf2_ros::Buffer transform_buffer_{ };
};

} // namespace make_goal
} // namespace umigv

#endif
