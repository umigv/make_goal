#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <nav_msgs/Odometry.h>
#include <string>
#include <fstream>

class MakeGoal {

public:
	MakeGoal(std::string fileName) {
		openFile(fileName);
		readIn();
	}

	void openFile(std::string fileName) {
		inputFile.open(fileName.c_str(), std::ifstream::in);
		if (!inputFile.is_open()) {
			std::cout << "Error: Failed to open " << fileName << std::endl;
			exit(1);
		}
	}
private:

	ros::NodeHandle n;
    move_base_msgs::MoveBaseGoal goal;
    ros::Publisher goalPub = n.advertise<move_base_msgs::MoveBaseGoal>("simple_goal", 1000);
    ros::Subscriber gpsSub = n.subscribe<nav_msgs::Odometry>("nav_msgs", 1000, &MakeGoal::readIn, this);
    std::ifstream inputFile;

	void makeAGoal(const double x, const double w);
    void readIn(const nav_msgs::Odometry::ConstPtr& msg);
    void readIn();

	// used for testing purposes
    void testMe();
};

void MakeGoal::makeAGoal(const double x, const double w) {
    goal.target_pose.header.frame_id = "make_goal";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.orientation.w = w;
    goalPub.publish(goal);
}

void MakeGoal::readIn(const nav_msgs::Odometry::ConstPtr& msg) {

	nav_msgs::Odometry::ConstPtr gpsPos = msg;

	double x;
    double w;

	if ((gpsPos->pose.pose.position.x == goal.target_pose.pose.position.x) &&
	    (gpsPos->pose.pose.orientation.w == goal.target_pose.pose.orientation.w)) {
		try {
			inputFile >> x >> w;
			makeAGoal(x, w);
		}
		catch (...) {
			std::cout << "Warning: Error reading coordinaties file. May have reached EOF." << std::endl;
		}
	}
	else {
		goal.target_pose.header.stamp = ros::Time::now();
		goalPub.publish(goal);
	}
    
}

void MakeGoal::readIn() {

	double x;
    double w;

	try {
		inputFile >> x >> w;
		makeAGoal(x, w);
	}
	catch (...) {
		std::cout << "Warning: Error reading coordinaties file. May have reached EOF." << std::endl;
	}	
}

void MakeGoal::testMe() {
	double x;
    double w;

	try {
		inputFile >> x >> w;
		makeAGoal(x, w);
	}
	catch (...) {
		std::cout << "Warning: Error reading coordinaties file. May have reached EOF." << std::endl;
	}

	while(true) {
		goalPub.publish(goal);
	}
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "make_goal");

	if (argc != 2) {
		std::cout << "Error: correct usage: ./make_goal [gps_coord_file.txt]" << std::endl;
		exit(1);
	}

	std::string filename = argv[1];

	MakeGoal make_goal(filename);

	ros::Rate loop_rate(10);
	
	ros::spin();
	//loop_rate.sleep();

	return 0;
}

