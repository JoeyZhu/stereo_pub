/*
 * stereo_pub_ros.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: joey
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

int main(int argc, char ** argv) {

    ros::init(argc, argv, "stereo_pub");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    //todo: get rid of compressedepth topic on image.
    image_transport::ImageTransport it(n);
    image_transport::Publisher left_image_pub = it.advertise(
            "camera/left/image", 1);
    image_transport::Publisher right_image_pub = it.advertise(
            "camera/right/image", 1);

    VideoCapture capture;
    Mat frame, left_img, right_img;

    int camera_id = 0;
    if(argc == 2){
        std::stringstream ss(argv[1]);
        ss >> camera_id;
    }

    if (!capture.open(camera_id)) {
        std::cout << "Capture from camera " << camera_id << " didn't work" << std::endl;
        return -1;
    }

    if (capture.isOpened()) {
        capture.set(CV_CAP_PROP_FRAME_WIDTH,2560);    //  1280x720
        capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
        capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
//        capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);  //640x480
//        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//        capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('Y','V','1','2'));
        //capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', 'U', 'Y', 'V'));

    } else {
        ROS_ERROR("camera is opened failed");
    }
    while (ros::ok()) {

        if (capture.isOpened()) {
            ROS_INFO_THROTTLE(1.0, "GET frame");
            capture >> frame;
            if( frame.empty() ){
                ROS_ERROR("get frame empty");
                break;
            }
            //cv::cvtColor(frame, frame, COLOR_BGR2GRAY);
            left_img = frame(Rect(0,0, frame.cols / 2, frame.rows));
            right_img = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

            sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(),
                    "bgr8", left_img).toImageMsg();
            sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(),
                    "bgr8", right_img).toImageMsg();
            left_image_pub.publish(left_msg);
            right_image_pub.publish(right_msg);

        }else{
            ROS_ERROR("camera offline");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
