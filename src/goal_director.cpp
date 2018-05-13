#include "goal_director.h"

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
GoalDirectorBuilder::with_threshold(const f64 threshold) noexcept {
    threshold_ = threshold;

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::with_goal_frame(std::string frame) noexcept {
    frame_ = std::move(frame);

    return *this;
}

GoalDirectorBuilder&
GoalDirectorBuilder::with_goal_id(std::string id) noexcept {
    id_ = std::move(id);

    return *this;
}

GoalDirector GoalDirectorBuilder::build() {
    if (!node_ || !is_ || !threshold_ || !frame_ || !id_) {
        throw std::logic_error{ "GoalDirectorBuilder::build" };
    }

    std::vector<GoalT> goals(std::istream_iterator<GoalT>{ *is_ },
                             std::istream_iterator<GoalT>{ });

    for (auto &goal : goals) {
    	goal.goal_id.id = id_.value();
    	goal.goal.target_pose.header.frame_id = frame_.value();
    }

    return { node_->advertise<GoalT>("move_base/goal", 10),
             std::move(goals), threshold_.value() };
}

GoalDirector::GoalDirector(GoalDirector &&other) noexcept
: publisher_{ std::move(other.publisher_) },
  goals_{ std::move(other.goals_) },
  distance_threshold_{ other.distance_threshold_ },
  current_iter_{ std::move(other.current_iter_) }, seq_{ other.seq_ } { }

void GoalDirector::update_odometry(
    const nav_msgs::Odometry::ConstPtr &odom_ptr
) {
    if (!current()) {
        return;
    }

    if (is_goal_reached(*odom_ptr)) {
        ++current_iter_;
    }
}

void GoalDirector::publish_goal(const ros::TimerEvent&) {
    if (!current()) {
        return;
    }

    GoalT to_broadcast = current().value();

    to_broadcast.header.seq = seq_;
    to_broadcast.goal.target_pose.header.seq = seq_;

    const auto now = ros::Time::now();
    to_broadcast.header.stamp = now;
    to_broadcast.goal.target_pose.header.stamp = now;
    publisher_.publish(to_broadcast);
    ++seq_;

    ROS_INFO_STREAM("published goal " << current_index().value());
}

tf2_ros::Buffer& GoalDirector::transform_buffer() noexcept {
    return transform_buffer_;
}

const tf2_ros::Buffer& GoalDirector::transform_buffer() const noexcept {
    return transform_buffer_;
}

GoalDirector::GoalDirector(ros::Publisher publisher,
                           std::vector<GoalT> goals,
                           const f64 threshold) noexcept
: publisher_{ std::move(publisher) }, goals_{ std::move(goals) },
  distance_threshold_{ threshold } { }

bool GoalDirector::is_goal_reached(const nav_msgs::Odometry &odom)
const noexcept {
    if (!current()) {
        return false;
    }

    const auto maybe_transformed = get_transformed_odom(odom);

    if (!maybe_transformed) {
        return false;
    }

    const auto &odom_pos = maybe_transformed.value().pose.pose.position;
    const auto &target_pos =
        current().value().get().goal.target_pose.pose.position;

    return distance(odom_pos, target_pos) <= distance_threshold_;
}

boost::optional<std::reference_wrapper<const GoalT>>
GoalDirector::current() const noexcept {
    if (current_iter_ < goals_.cbegin() || current_iter_ >= goals_.cend()) {
        return boost::none;
    }

    using ReturnT = boost::optional<std::reference_wrapper<const GoalT>>;
    return ReturnT{ std::cref(*current_iter_) };
}

boost::optional<nav_msgs::Odometry>
GoalDirector::get_transformed_odom(const nav_msgs::Odometry &odom)
const noexcept {
    if (!current()) {
        return boost::none;
    }

    nav_msgs::Odometry transformed;
    const auto &source = odom.header.frame_id;
    const auto &target =
        current().value().get().goal.target_pose.header.frame_id;

    try {
        transform_buffer().transform(odom, transformed, target,
                                     ros::Duration{ 0.1 });
    } catch (const tf2::TransformException &e) {
        ROS_WARN_STREAM("GoalDirector::get_transformed_odom: unable to "
                        "transform from '" << source << "' to '" << target
                        << "': " << e.what());
        return boost::none;
    }

    using ReturnT = boost::optional<nav_msgs::Odometry>;
    return ReturnT{ transformed };
}

boost::optional<std::size_t> GoalDirector::current_index() const noexcept {
    if (!current()) {
        return boost::none;
    }

    const auto index = std::distance(goals_.cbegin(), current_iter_);

    return { static_cast<std::size_t>(index) };
}

} // namespace make_goal
} // namespace umigv
