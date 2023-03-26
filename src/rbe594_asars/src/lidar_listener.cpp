// https://github.com/wonwon0/gazebo_contact_republisher

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void dataCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	ROS_INFO("LaserScan: (val,angle)-(%f,%f)", msg->range_min, msg->angle_min);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "lidar_listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan", 1000, dataCallback);
	ros::spin();
	return 0;
}