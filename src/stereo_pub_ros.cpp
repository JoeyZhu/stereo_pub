/*
 * stereo_pub_ros.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: joey
 */



#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char ** argv){

	ros::init(argc, argv, "stereo_pub");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	  image_transport::ImageTransport it(n);
	  image_transport::Publisher left_image_pub = it.advertise("camera/left/image", 1);
	  image_transport::Publisher right_image_pub = it.advertise("camera/right/image", 1);



	while(ros::ok()){

		//todo: read image from camera3
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

		ROS_INFO("hello");
		ros::spinOnce();
		loop_rate.sleep();
	}
}
