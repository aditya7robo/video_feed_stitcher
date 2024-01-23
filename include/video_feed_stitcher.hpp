#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

class ImageConcatenator
{
    public:
    ImageConcatenator(ros::NodeHandle& nh);

    void imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg);
    void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat cropBlackBars(const cv::Mat &image);
    void concatenateImages();

    private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_left_;
    image_transport::Subscriber sub_image_right_;
    image_transport::Publisher pub_image_;

    cv::Mat image_left_;
    cv::Mat image_right_;
};

// Callback for the first camera image topic subscriber
// void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
// {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         // cv_ptr->image now contains the image from the first camera
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     mergeImages(cv_ptr->image); // Pass the image to the merging function
// }

// // Callback for the second camera image topic subscriber
// void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
// {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         // cv_ptr->image now contains the image from the second camera
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     mergeImages(cv_ptr->image); // Pass the image to the merging function
// }
// ```

// ### Step 2: Merge the Images Pixel-wise

// ```cpp
// void mergeImages(cv::Mat& image1, cv::Mat& image2)
// {
//     // Assuming image1 and image2 have the same size and type
//     cv::Mat merged_image(image1.rows, image1.cols, image1.type());

//     for (int i = 0; i < image1.rows; ++i)
//     {
//         for (int j = 0; j < image1.cols; ++j)
//         {
//             // Merge pixels from image1 and image2 into merged_image pixel-wise
//             // This example simply takes an average of the two. You can modify this logic based on your merge requirements.
//             merged_image.at<cv::Vec3b>(i, j) = (image1.at<cv::Vec3b>(i, j) / 2 + image2.at<cv::Vec3b>(i, j) / 2);
//         }
//     }

//     // Publish the merged image
//     publishMergedImage(merged_image);
// }
// ```

// ### Step 3: Publish the Merged Image

// ```cpp
// ros::Publisher pub;

// void publishMergedImage(const cv::Mat& merged_image)
// {
//     // Convert the merged OpenCV image back to a ROS image message
//     cv_bridge::CvImage out_msg;
//     out_msg.header.stamp = ros::Time::now(); // Use the header from one of the original images if necessary
//     out_msg.encoding = sensor_msgs::image_encodings::BGR8;
//     out_msg.image = merged_image;

//     // Publish the merged image
//     pub.publish(out_msg.toImageMsg());
// }
// ```
