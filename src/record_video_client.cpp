#include <ros/ros.h>
#include "streaming_media/RecordVideo.h"  // streaming_media 包名

int main(int argc, char** argv) {
    ros::init(argc, argv, "record_video_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<streaming_media::RecordVideo>("/record_video");
    streaming_media::RecordVideo srv;

    srv.request.record = true;  // Set to true to record video
    if (client.call(srv)) {
        ROS_INFO("Video capture: %s", srv.response.result.c_str());
    } else {
        ROS_ERROR("Failed to call service /record_video");
    }

    return 0;
}
