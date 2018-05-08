// Code refactored by Michael specifically for Greg

#include "simple_goal.h"
#include "goal_director.h"

#include <fstream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <umigv_utilities/exceptions.hpp>
#include <umigv_utilities/rosparam.hpp>
#include <umigv_utilities/types.hpp>
#include <umigv_utilities/utility.hpp>

using namespace umigv::types;

using umigv::make_goal::ActionGoalT;
using umigv::make_goal::GoalDirector;
using umigv::make_goal::GoalDirectorBuilder;

static ActionGoalT make_action_goal_pattern(const std::string &frame_id,
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
	f64 threshold;
	ros::Rate rate = 1.0;
};

static Parameters get_parameters(ros::NodeHandle &node) {
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
			umigv::get_parameter_fatal<f64>(node, "threshold");
	} catch (const umigv::ParameterNotFoundException &e) {
		ROS_FATAL_STREAM("unable to find parameter '" << e.parameter() << "'");
		umigv::blocking_shutdown();
	}

	params.rate = ros::Rate{ umigv::get_parameter_or(node, "rate"s, 1.0) };

	return params;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "make_goal");

	ros::NodeHandle node;
	ros::NodeHandle private_node{ "~" };

	const Parameters params = get_parameters(private_node);

	const auto pattern =
		make_action_goal_pattern(params.frame_id, params.goal_id);
	std::ifstream ifs{ params.goals_filename };

	GoalDirector director =
		GoalDirectorBuilder{ }.with_node(node)
							  .from_stream(ifs)
							  .with_pattern(pattern)
							  .with_threshold(params.threshold)
							  .build();

	const auto subscriber =
		node.subscribe<nav_msgs::Odometry>("odom", 10,
										   &GoalDirector::update_odometry,
										   &director);
	const auto timer =
		node.createTimer(params.rate, &GoalDirector::publish_goal, &director);

	ros::spin();
}
