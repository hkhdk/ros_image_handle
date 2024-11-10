#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "streaming_media/TakePhoto.h"  // streaming_media 包名
#include <string>

std::string base_save_photo_path = "/home/images/";

bool saveImage(const cv::Mat& image, const std::string& filename, std::string& path) {
    std::string full_path = base_save_photo_path + path + filename;
    if (!cv::imwrite(full_path, image)) {
        ROS_ERROR("Failed to save image.");
        return false;
    }
    return true;
}

bool takePhotoCallback(streaming_media::TakePhoto::Request  &req, streaming_media::TakePhoto::Response &res) {
    bool is_rgb_success = false, is_depth_success = false;
    if (req.record) {  // take photo(a frame)
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber rgb_sub = it.subscribe("/usb_cam/image_raw", 5, [&is_rgb_success](const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            std::string rgb_path = "rgb/";
            if (saveImage(cv_ptr->image, "rgb_image.jpg", rgb_path)) {
                ROS_INFO("RGB image saved to: %s", rgb_path.c_str());
                is_rgb_success = true;
            } else ROS_ERROR("Failed to save rgb image.");
        });

        image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_raw", 5, [&is_depth_success](const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono16");
            std::string depth_path = "depth/";
            if (saveImage(cv_ptr->image, "depth_image.png", depth_path)) {
                ROS_INFO("Depth image saved to: %s", depth_path.c_str());
                is_depth_success = true;
            } else ROS_ERROR("Failed to save depth image.");
        });

        ros::Duration(10.0).sleep();
        ros::spinOnce();

        if (is_rgb_success && is_depth_success) {
            res.result = base_save_photo_path;  // Return the folder path
        } else res.result = "false";
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "take_photo_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/take_photo", takePhotoCallback);
    ROS_INFO("Ready to take photos.");
    ros::spin();

    return 0;
}

