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
    move_base_msgs::MoveBaseActionGoal goal;
    ros::Publisher goalPub = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
    ros::Subscriber gpsSub = n.subscribe<nav_msgs::Odometry>("odom", 1000, &MakeGoal::readIn, this);
    std::ifstream inputFile;

	void makeAGoal(const double px, const double py, const double pz, const double ox, const double oy, const double oz, const double ow);
    void readIn(const nav_msgs::Odometry::ConstPtr& msg);
    void readIn();

	// used for testing purposes
    void testMe();
};

// edit to match input coordinates
void MakeGoal::makeAGoal(const double px, const double py, const double pz, const double ox, const double oy,
	const double oz, const double ow) {
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal.target_pose.header.stamp = ros::Time::now();
    goal.goal.target_pose.pose.position.x = px;
    goal.goal.target_pose.pose.position.y = py;
    goal.goal.target_pose.pose.position.z = pz;
    goal.goal.target_pose.pose.orientation.x = ox;
    goal.goal.target_pose.pose.orientation.y = oy;
    goal.goal.target_pose.pose.orientation.z = oz;
    goal.goal.target_pose.pose.orientation.w = ow;
    goalPub.publish(goal);
}

void MakeGoal::readIn(const nav_msgs::Odometry::ConstPtr& msg) {

	nav_msgs::Odometry::ConstPtr gpsPos = msg;

	// read in files according to coordinates being used
	// positon coords
	double px, py, pz;
    //orientation coords
    double ox, oy, oz, ow;

	if ((gpsPos->pose.pose.position.x == goal.goal.target_pose.pose.position.x) &&
		(gpsPos->pose.pose.position.y == goal.goal.target_pose.pose.position.y) &&
		(gpsPos->pose.pose.position.z == goal.goal.target_pose.pose.position.z) &&
	    (gpsPos->pose.pose.orientation.x == goal.goal.target_pose.pose.orientation.x) &&
		(gpsPos->pose.pose.orientation.y == goal.goal.target_pose.pose.orientation.y) &&
		(gpsPos->pose.pose.orientation.z == goal.goal.target_pose.pose.orientation.z) &&
		(gpsPos->pose.pose.orientation.w == goal.goal.target_pose.pose.orientation.w)) {
		try {
			// edit to match input coordinates
			inputFile >> px >> py >> pz >> ox >> oy >> oz >> ow;
			makeAGoal(px, py, pz, ox, oy, oz, ow);
		}
		catch (...) {
			std::cout << "Warning: Error reading coordinaties file. May have reached EOF." << std::endl;
		}
	}
	
	else {
		goal.goal.target_pose.header.stamp = ros::Time::now();
		goalPub.publish(goal);
	}
	
    
}

void MakeGoal::readIn() {

	// positon coords
	double px, py, pz;
    //orientation coords
    double ox, oy, oz, ow;

	try {
		inputFile >> px >> py >> pz >> ox >> oy >> oz >> ow;
		makeAGoal(px, py, pz, ox, oy, oz, ow);
	}
	catch (...) {
		std::cout << "Warning: Error reading coordinaties file. May have reached EOF." << std::endl;
	}	
}

void MakeGoal::testMe() {
	// positon coords
	double px, py, pz;
    //orientation coords
    double ox, oy, oz, ow;

	try {
		inputFile >> px >> py >> pz >> ox >> oy >> oz >> ow;
		makeAGoal(px, py, pz, ox, oy, oz, ow);
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

