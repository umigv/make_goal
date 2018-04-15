// Code refactored by Michael specifically for Greg

#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <fstream>
#include <cmath>
#include <deque>

class ROSNode {
private:

	class MakeGoal {
		friend class ROSNode;
	public:
		MakeGoal(const std::string& fileName) : inputFile(fileName.c_str()) {
			fillWayPoints();
		}

		bool wayPointsEmpty();
		void set_gpsPos(const nav_msgs::Odometry::ConstPtr& msg);
		move_base_msgs::MoveBaseActionGoal& getNextWaypoint();
	    move_base_msgs::MoveBaseActionGoal& getCurrWaypoint();

	private:
	    nav_msgs::Odometry::ConstPtr gpsPos;
	    std::ifstream inputFile;
	    std::deque<move_base_msgs::MoveBaseActionGoal> wayPoints;

	    // allowed variation from actual goal coordination
	    double threshold = 0.1;

	    void fillWayPoints();
	    bool reachedWayPoint();
	    
	};

public:
	ROSNode(const std::string& filename) {
		make_goal = new MakeGoal(filename);
		initGoal();
	}

	void publishGoal(const nav_msgs::Odometry::ConstPtr& msg);
	void initGoal();

	~ROSNode() {
		delete make_goal;
	}

	MakeGoal* make_goal;

private:

	ros::NodeHandle n;
	int loop = 0;
	ros::Publisher goalPub = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
	ros::Subscriber gpsSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &ROSNode::publishGoal, this);

};

move_base_msgs::MoveBaseActionGoal& ROSNode::MakeGoal::getNextWaypoint() {
	wayPoints.pop_front();

	// assumes this will shut down the node and thus the code will not segfault
	if (wayPointsEmpty()) {
		ros::shutdown();
	}

	return wayPoints.front();
}



move_base_msgs::MoveBaseActionGoal& ROSNode::MakeGoal::getCurrWaypoint() {
	return wayPoints.front();
}



void ROSNode::publishGoal(const nav_msgs::Odometry::ConstPtr& msg) {
	make_goal->set_gpsPos(msg);

	if (make_goal->reachedWayPoint()) {

		std::cout << "Waypoint: " << loop << std::endl;
		++loop;

		move_base_msgs::MoveBaseActionGoal nextWaypoint = make_goal->getNextWaypoint();
		nextWaypoint.goal.target_pose.header.stamp = ros::Time::now();

		goalPub.publish(nextWaypoint);
		
	}
	else {
		move_base_msgs::MoveBaseActionGoal currWaypoint = make_goal->getCurrWaypoint();
		currWaypoint.goal.target_pose.header.stamp = ros::Time::now();

		goalPub.publish(currWaypoint);
	}
}



void ROSNode::initGoal() {
	move_base_msgs::MoveBaseActionGoal currWaypoint = make_goal->getCurrWaypoint();
	currWaypoint.goal.target_pose.header.stamp = ros::Time::now();
	//std::cout << "Waypoint: " << loop << std::endl;
	++loop;

	goalPub.publish(currWaypoint);
}



void ROSNode::MakeGoal::set_gpsPos(const nav_msgs::Odometry::ConstPtr& msg) {
	this->gpsPos = msg; 
}



bool ROSNode::MakeGoal::wayPointsEmpty() {
	return wayPoints.empty();
}



void ROSNode::MakeGoal::fillWayPoints() {
	if (!inputFile.is_open()) {
		std::cout << "Error: Failed to open read file" << std::endl;
		exit(1);
	}

	// positon coords
	double px, py, pz;
    //orientation coords
    double ox, oy, oz, ow;

	while (inputFile.good()) {
		// edit to match input coordinates
		inputFile >> px >> py >> pz >> ox >> oy >> oz >> ow;

		move_base_msgs::MoveBaseActionGoal goal;

		goal.goal.target_pose.header.frame_id = "map";
	    // goal.goal.target_pose.header.stamp = ros::Time::now();
	    goal.goal.target_pose.pose.position.x = px;
	    goal.goal.target_pose.pose.position.y = py;
	    goal.goal.target_pose.pose.position.z = pz;
	    goal.goal.target_pose.pose.orientation.x = ox;
	    goal.goal.target_pose.pose.orientation.y = oy;
	    goal.goal.target_pose.pose.orientation.z = oz;
	    goal.goal.target_pose.pose.orientation.w = ow;

		this->wayPoints.push_back(goal);
	}

	inputFile.close();

}



bool ROSNode::MakeGoal::reachedWayPoint() {

	move_base_msgs::MoveBaseActionGoal currWP = getCurrWaypoint();

	if ((abs(gpsPos->pose.pose.position.x - currWP.goal.target_pose.pose.position.x) <= threshold) &&
		(abs(gpsPos->pose.pose.position.y - currWP.goal.target_pose.pose.position.y) <= threshold) &&
		(abs(gpsPos->pose.pose.position.z - currWP.goal.target_pose.pose.position.z) <= threshold)) {
		return true;
	}
	return false;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "make_goal");


	if (argc != 2) {
		std::cout << "Error: correct usage: ./make_goal [gps_coord_file.txt]" << std::endl;
		exit(1);
	}

	std::string filename = argv[1];
	ROSNode ros_node(filename);
	
	while(ros::ok()) {
		ros::spin();
	}

	return 0;
}

