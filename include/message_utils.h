#ifndef UMIGV_MAKE_GOAL_MESSAGE_UTILS_H
#define UMIGV_MAKE_GOAL_MESSAGE_UTILS_H

#include <iostream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
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
