#include <ros/ros.h>
#include "streaming_media/TakePhoto.h"  // streaming_media 包名

int main(int argc, char** argv) {
    ros::init(argc, argv, "take_photo_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<streaming_media::TakePhoto>("/take_photo");
    streaming_media::TakePhoto srv;

    srv.request.record = true;  // Set to true to take a photo
    if (client.call(srv)) {
        ROS_INFO("Photo taken: %s", srv.response.result.c_str());
    } else {
        ROS_ERROR("Failed to call service /take_photo");
    }
    ros::spin();

    return 0;
}
