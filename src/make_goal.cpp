// Code refactored by Michael specifically for Greg

#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>

#include <boost/optional.hpp>

#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <umigv_utilities/rosparam.hpp>
#include <umigv_utilities/utility.hpp>

namespace geometry_msgs {

std::istream& operator>>(std::istream &is, Point &point) {
	return is >> point.x >> point.y >> point.z;
}

std::istream& operator>>(std::istream &is, Quaternion &quat) {
	return is >> quat.x >> quat.y >> quat.z >> quat.w;
}

std::istream& operator>>(std::istream &is, Pose &pose) {
	return is >> pose.position >> pose.orientation;
}

Point operator-(const Point &lhs, const Point &rhs) {
	Point difference;

	difference.x = lhs.x - rhs.x;
	difference.y = lhs.y - rhs.y;
	difference.z = lhs.z - rhs.z;

	return difference;
}

} // namespace geometry_msgs

struct SimpleGoal {
	geometry_msgs::Pose pose;
};

using ActionGoalT = move_base_msgs::MoveBaseActionGoal;

ActionGoalT as_action_goal(const SimpleGoal &goal,
					       ActionGoalT pattern) noexcept {
	pattern.goal.target_pose.pose = goal.pose;

	return pattern;
}

std::istream& operator>>(std::istream &is, SimpleGoal &goal) {
	return is >> goal.pose;
}

// euclidean norm of Point
inline double mag(const geometry_msgs::Point &point) noexcept {
	return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

class SimpleGoalIterator {
public:
	SimpleGoalIterator(std::istream &is)
	: goals_(std::istream_iterator<SimpleGoal>{ is },
			 std::istream_iterator<SimpleGoal>{ }) { }

	boost::optional<std::reference_wrapper<const SimpleGoal>>
	next() noexcept {
		if (current_iter_ >= goals_.cend()) {
			return boost::none;
		}

		++current_iter_;
		return current();
	}

	boost::optional<std::reference_wrapper<const SimpleGoal>>
	current() const noexcept {
		if (current_iter_ >= goals_.cend()) {
			return boost::none;
		}

		return { std::cref(*current_iter_) };
	}

	std::size_t num_goals() const noexcept {
		return goals_.size();
	}

private:
	std::vector<SimpleGoal> goals_;
	std::vector<SimpleGoal>::const_iterator current_iter_ = goals_.cbegin();
};

class GoalDirector {
public:
	GoalDirector(ros::NodeHandle &node, SimpleGoalIterator goals,
				 ActionGoalT goal_pattern, const double threshold = 0.5)
	: publisher_{ node.advertise<ActionGoalT>("move_base/goal", 10) },
	  goals_{ std::move(goals) },
	  goal_pattern_{ std::move(goal_pattern) },
	  distance_threshold_{ threshold } {
		publish_current_goal();
	}

	void odometry_callback(const nav_msgs::Odometry::ConstPtr &odom_ptr) {
		const geometry_msgs::Pose &pose = odom_ptr->pose.pose;

		if (is_reached(pose)) {
			publish_next_goal();
		}
	}

	void force_publish(const ros::TimerEvent&) {
		publish_current_goal();
	}

private:
	bool is_reached(const geometry_msgs::Pose &pose) const noexcept {
		const auto current = goals_.current();

		if (!current) {
			return false;
		}

		const auto &current_position = current.value().get().pose.position;

		return mag(current_position - pose.position) < distance_threshold_;
	}

	void publish_current_goal() {
		const auto maybe_current = goals_.current();

		if (!maybe_current) {
			return;
		}

		publish_goal(maybe_current.value());
	}

	void publish_next_goal() {
		const auto maybe_next = goals_.next();

		if (!maybe_next) {
			return;
		}

		++goal_index_;
		publish_goal(maybe_next.value());
	}

	void publish_goal(const SimpleGoal &goal) {
		const auto now = ros::Time::now();

		goal_pattern_.header.stamp = now;
		goal_pattern_.goal.target_pose.header.stamp = now;
		publisher_.publish(as_action_goal(goal, goal_pattern_));
		++goal_pattern_.header.seq;
		++goal_pattern_.goal.target_pose.header.seq;

		ROS_INFO_STREAM("published goal " << goal_index_);
	}

	ros::Publisher publisher_;
	SimpleGoalIterator goals_;
	ActionGoalT goal_pattern_;
	double distance_threshold_;
	std::size_t goal_index_ = 0;
};

ActionGoalT make_action_goal_pattern(const std::string &frame_id,
									 const std::string &goal_id) {
	ActionGoalT pattern;

	pattern.header.seq = 0;

	pattern.goal_id.id = goal_id;

	pattern.goal.target_pose.header.seq = 0;
	pattern.goal.target_pose.header.frame_id = frame_id;

	return pattern;
}

struct Parameters {
	std::string frame_id;
	std::string goal_id;
	std::string goals_filename;
	double threshold;
};

Parameters get_parameters(ros::NodeHandle &node) {
	using namespace std::literals;

	Parameters params;

	try {
		params.frame_id =
			umigv::get_parameter_fatal<std::string>(node, "frame_id"s);
		params.goal_id =
			umigv::get_parameter_fatal<std::string>(node, "goal_id"s);
		params.goals_filename =
			umigv::get_parameter_fatal<std::string>(node, "goals_filename"s);
		params.threshold =
			umigv::get_parameter_fatal<double>(node, "threshold");
	} catch (const umigv::ParameterNotFoundException &e) {
		ROS_FATAL_STREAM("unable to find parameter '" << e.parameter() << "'");
		umigv::blocking_shutdown();
	}

	return params;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "make_goal");

	ros::NodeHandle node;
	ros::NodeHandle private_node{ "~" };

	const Parameters params = get_parameters(private_node);

	const auto goal_pattern =
		make_action_goal_pattern(params.frame_id, params.goal_id);
	std::ifstream ifs(params.goals_filename);
	SimpleGoalIterator goals{ ifs };

	GoalDirector director(node, std::move(goals),
						  goal_pattern, params.threshold);

	const auto subscriber =
		node.subscribe<nav_msgs::Odometry>("odom", 10,
										   &GoalDirector::odometry_callback,
										   &director);
	const auto timer =
		node.createTimer(ros::Rate{ 1.0 }, &GoalDirector::force_publish,
						 &director);

	ros::spin();
}
