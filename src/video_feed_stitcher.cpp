#include<video_feed_stitcher.hpp>

ImageConcatenator::ImageConcatenator(ros::NodeHandle& nh) : it_(nh_)
{
    //Subscribers
    sub_image_left_upright_ = it_.subscribe("left_image_upright", 1, &ImageConcatenator::imageUprightCallbackLeft, this);
    sub_image_right_upright_ = it_.subscribe("right_image_upright", 1, &ImageConcatenator::imageUprightCallbackRight, this);
    
    //Publishers
    pub_image_ = it_.advertise("stitched_images", 1);
}

void ImageConcatenator::imageUprightCallbackLeft(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("Entered left callback.");
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);//"rgb8");
        image_left_ = cv_ptr->image;
        concatenateImages();
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void ImageConcatenator::imageUprightCallbackRight(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("Entered right callback.");
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);//"rgb8");
        image_right_ = cv_ptr->image;
        concatenateImages();
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

cv::Mat ImageConcatenator::cropBlackBars(const cv::Mat &image) 
{
    //Create a binary mask using inRange with a low threshold to capture the black bars.
    cv::Mat mask;
    cv::inRange(image, cv::Scalar(0, 0, 0), cv::Scalar(15, 15, 15), mask);
    
    //Find the bounding rectangle of the non-zero area (not black) in the mask.
    cv::Mat points;
    cv::findNonZero(~mask, points); // Use inversion (~) of the mask to find non-black areas.
    cv::Rect roi = cv::boundingRect(points);

    // ROS_INFO("[Video Stitcher] Cropping Done!");

    return image(roi); //Crop the image using the bounding rectangle.

    // // Initial large values for cropping
    // int top = image.rows, bottom = -1, left = image.cols, right = -1;

    // double low_threshold = 15; //Calculate a threshold that would be considered "black".
    
    // //Examine rows for consistent low intensity indicating a black bar.
    // for (int i = 0; i < image.rows; ++i) {
    //     if (cv::mean(image.row(i))[0] < low_threshold) {
    //         if (i < image.rows / 2 && i < top) {
    //             top = i;
    //         } else if (i >= image.rows / 2 && (i > bottom || bottom == -1)) {
    //             bottom = i;
    //         }
    //     }
    // }

    // //Create the largest ROI excluding detected black bars
    // if (top >= image.rows || bottom <= 0 || left >= image.cols || right <= 0) {
    //     //If no black bars found, return the original image.
    //     return image.clone();
    // } else {
    //     //If black bars detected, bottom and right will be the starting points of the black regions.
    //     cv::Rect roi(0, top, image.cols, (bottom > 0 ? bottom : image.rows) - top);
    //     return image(roi);
    // }
}


void ImageConcatenator::concatenateImages()
{
    // ROS_INFO("Entered stitcher.");
    if (!image_left_.empty() && !image_right_.empty())
    {
        // ROS_INFO("Images not empty.");
        //Eliminate Letterboxing.
        cv::Mat cropped_image_left_ = cropBlackBars(image_left_);
        cv::Mat cropped_image_right_ = cropBlackBars(image_right_);

        //Rotate images 90 deg to the left.
        cv::Mat rotated_image_left, rotated_image_right;
        cv::Point2f center1(cropped_image_left_.cols / 2.0, cropped_image_left_.rows / 2.0);
        cv::Point2f center2(cropped_image_right_.cols / 2.0, cropped_image_right_.rows / 2.0);

        //Get rotation matrices for rotating the images around their center.
        cv::Mat rot_mat_left = cv::getRotationMatrix2D(center1, 90, 1.0);
        cv::Mat rot_mat_right = cv::getRotationMatrix2D(center2, 90, 1.0);

        //Perform rotation.
        cv::warpAffine(cropped_image_left_, rotated_image_left, rot_mat_left, cropped_image_left_.size());
        cv::warpAffine(cropped_image_right_, rotated_image_right, rot_mat_right, cropped_image_right_.size());

        cv::Mat stitched;
        cv::hconcat(cropped_image_left_, cropped_image_right_, stitched);
        // cv::hconcat(image_left_, image_right_, stitched);

        // Convert back to ROS message and publish.
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", stitched).toImageMsg();
        pub_image_.publish(out_msg);
        // ROS_INFO("Stitching done.");
    }
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_concatenator");
    ros::NodeHandle nh;
    ImageConcatenator ic(nh);
    ros::spin();
    return 0;
}