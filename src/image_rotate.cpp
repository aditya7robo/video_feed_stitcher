#include <image_rotate.hpp>

ImageRotator::ImageRotator(ros::NodeHandle& nh) : it_(nh_)
{
    //Subscribers
    sub_image_left_ = it_.subscribe("/sensor_d415_right/color/image_raw", 1, &ImageRotator::imageCallbackLeft, this);
    sub_image_right_ = it_.subscribe("/sensor_d415_left/color/image_raw", 1, &ImageRotator::imageCallbackRight, this);
   
    //Publishers
    pub_image_left_upright_ = it_.advertise("left_image_upright", 1);
    pub_image_right_upright_ = it_.advertise("right_image_upright", 1);
}

void ImageRotator::imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("Entered left callback.");
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        cv::Point2f center1(cv_ptr->image.cols / 2.0, cv_ptr->image.rows / 2.0);
        cv::Mat rot_mat_left = cv::getRotationMatrix2D(center1, 90, 1.0);
        cv::warpAffine(cv_ptr->image, cv_ptr->image, rot_mat_left, cv_ptr->image.size());
        cv::Mat image_left_temp_ = cv_ptr->image;
        sensor_msgs::ImagePtr out_msg_l = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_left_temp_).toImageMsg();
        pub_image_left_upright_.publish(out_msg_l);
        // cv::Mat image_left_ = cv_ptr->image;
        // rotateImages();
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void ImageRotator::imageCallbackRight(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("Entered right callback.");
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        cv::Point2f center2(cv_ptr->image.cols / 2.0, cv_ptr->image.rows / 2.0);
        cv::Mat rot_mat_right = cv::getRotationMatrix2D(center2, 90, 1.0);
        cv::warpAffine(cv_ptr->image, cv_ptr->image, rot_mat_right, cv_ptr->image.size());
        cv::Mat image_right_temp_ = cv_ptr->image;
        sensor_msgs::ImagePtr out_msg_r = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_right_temp_).toImageMsg();
        pub_image_right_upright_.publish(out_msg_r);
        // cv::Mat image_right_ = cv_ptr->image;
        // rotateImages();
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void ImageRotator::rotateImages()
{
    ROS_INFO("Entered rotator.");
    if (!image_left_.empty() && !image_right_.empty())
    {
        ROS_INFO("Images not empty.");

        //Rotate images 90 deg to the left.
        cv::Mat rotated_image_left, rotated_image_right;
        cv::Point2f center1(image_left_.cols / 2.0, image_left_.rows / 2.0);
        cv::Point2f center2(image_right_.cols / 2.0, image_right_.rows / 2.0);

        //Get rotation matrices for rotating the images around their center.
        cv::Mat rot_mat_left = cv::getRotationMatrix2D(center1, 90, 1.0);
        cv::Mat rot_mat_right = cv::getRotationMatrix2D(center2, 90, 1.0);

        //Perform rotation.
        cv::warpAffine(image_left_, rotated_image_left, rot_mat_left, image_left_.size());
        cv::warpAffine(image_right_, rotated_image_right, rot_mat_right, image_right_.size());


        // Convert back to ROS message and publish.
        sensor_msgs::ImagePtr out_msg_l = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rotated_image_left).toImageMsg();
        pub_image_left_upright_.publish(out_msg_l);
        sensor_msgs::ImagePtr out_msg_r = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rotated_image_right).toImageMsg();
        pub_image_right_upright_.publish(out_msg_r);
        ROS_INFO("Rotating done.");
    }
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_rotator");
    ros::NodeHandle nh;
    ImageRotator ir(nh);
    ros::spin();
    return 0;
}