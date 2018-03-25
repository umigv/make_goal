# make_goal
make_goal ROS package


Subscribes to topic "nav_msgs" and expects a "nav_msgs/Odometry" input.
Publishes to topic "simple_goal" a "move_base_msgs/MoveBaseGoal" output.

!Compile
Compile in ROS lunar with catkin_make
$catkin_make

!Run
$rosrun make_goal make_goal [gps_coord_file.txt]
