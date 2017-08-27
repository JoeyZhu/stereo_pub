/*
 * stereo_pub_ros.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: joey
 */



#include "ros/ros.h"


int main(int argc, char ** argv){

	ros::init(argc, argv, "stereo_pub");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	while(ros::ok()){
		ROS_INFO("hello");
		ros::spinOnce();
		loop_rate.sleep();
	}
}
