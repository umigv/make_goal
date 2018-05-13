#ifndef UMIGV_MAKE_GOAL_MESSAGE_UTILS_H
#define UMIGV_MAKE_GOAL_MESSAGE_UTILS_H

#include <iostream>
#include <utility>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <umigv_utilities/types.hpp>

namespace geometry_msgs {

std::ostream& operator<<(std::ostream &os, const Point &point);

std::ostream& operator<<(std::ostream &os, const Quaternion &quat);

std::ostream& operator<<(std::ostream &os, const Pose &pose);

std::istream& operator>>(std::istream &is, Point &point);

std::istream& operator>>(std::istream &is, Quaternion &quat);

std::istream& operator>>(std::istream &is, Pose &pose);

Point operator-(const Point &point) noexcept;

Point operator+(const Point &lhs, const Point &rhs) noexcept;

Point operator-(const Point &lhs, const Point &rhs) noexcept;

} // namespace geometry_msgs

namespace move_base_msgs {

std::ostream& operator<<(std::ostream &os, const MoveBaseActionGoal &goal);

std::istream& operator>>(std::istream &is, MoveBaseActionGoal &goal);

} // namespace move_base_msgs

namespace tf2 {

template <>
inline const std::string&
getFrameId(const nav_msgs::Odometry &odom) {
    return odom.header.frame_id;
}

template <>
inline const ros::Time&
getTimestamp(const nav_msgs::Odometry &odom) {
    return odom.header.stamp;
}

template <>
inline void doTransform(
    const nav_msgs::Odometry &in, nav_msgs::Odometry &out,
    const geometry_msgs::TransformStamped &transform
) {
    geometry_msgs::PoseWithCovarianceStamped in_pose;

    in_pose.header = in.header;
    in_pose.pose = in.pose;

    geometry_msgs::PoseWithCovarianceStamped out_pose;

    out_pose.header = out.header;
    out_pose.pose = out.pose;

    doTransform(in_pose, out_pose, transform);

    out.pose = std::move(out_pose.pose);
    out.header = std::move(out_pose.header);
}

} // namespace tf2

namespace umigv {
namespace make_goal {

f64 dot(const geometry_msgs::Point &lhs,
        const geometry_msgs::Point &rhs) noexcept;

f64 norm(const geometry_msgs::Point &point) noexcept;

f64 distance(const geometry_msgs::Point &lhs,
             const geometry_msgs::Point &rhs) noexcept;

} // namespace make_goal
} // namespace umigv

#endif
