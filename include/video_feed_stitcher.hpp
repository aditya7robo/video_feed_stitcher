#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

class ImageConcatenator
{
    public:
    ImageConcatenator(ros::NodeHandle& nh);
    
    void imageUprightCallbackLeft(const sensor_msgs::ImageConstPtr& msg);
    void imageUprightCallbackRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat cropBlackBars(const cv::Mat &image);
    void concatenateImages();

    private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_left_;
    image_transport::Subscriber sub_image_right_;
    image_transport::Publisher pub_image_;

    image_transport::Publisher pub_image_left_upright_;
    image_transport::Publisher pub_image_right_upright_;
    image_transport::Subscriber sub_image_left_upright_;
    image_transport::Subscriber sub_image_right_upright_;

    cv::Mat image_left_;
    cv::Mat image_right_;
};

