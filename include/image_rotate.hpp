#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

class ImageRotator
{
    public:
    ImageRotator(ros::NodeHandle& nh);

    void imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg);
    void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg);
    void rotateImages();

    private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::Publisher pub_image_left_upright_;
    image_transport::Publisher pub_image_right_upright_;
    image_transport::Subscriber sub_image_left_;
    image_transport::Subscriber sub_image_right_;

    cv::Mat image_left_;
    cv::Mat image_right_;
};