#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <streaming_media/RecordVideo.h>  // streaming_media 包名
#include <std_msgs/Bool.h>
#include <mutex>
#include <vector>
#include <thread>

bool recording = false, isTimeOut = false;
std::string base_save_video_path = "/home/videos/";
image_transport::Subscriber sub_rgb;
image_transport::Subscriber sub_depth;
cv::VideoWriter video_writer_rgb, video_writer_depth;
std::vector<cv::Mat> framesOfRGB, framesOfdepth;


// 视频捕获回调函数
void videoCaptureCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic_name) {
    if (!recording) return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        if (topic_name == "rgb") {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            framesOfRGB.emplace_back(cv_ptr->image);
        } else if (topic_name == "depth") {
            cv_ptr = cv_bridge::toCvCopy(msg, "mono16");
            framesOfdepth.emplace_back(cv_ptr->image);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// 写入视频文件
bool write_video_frame_into_file(bool isTimeOut) {
    bool is_rgb_success = false, is_depth_success = false;
    //std::lock_guard<std::mutex> lock(video_mutex); // 构造的时候自动锁定，析构的时候自动释放

    std::string rgbVideoFileName = isTimeOut ? "rgb_video.mp4" : "rgb_video_timeout.mp4";
    std::string depthVideoFileName = isTimeOut ? "depth_video.mp4" : "depth_video_timeout.mp4";

    if (!framesOfRGB.empty()) {
        cv::VideoWriter video_writer_rgb(base_save_video_path + rgbVideoFileName, 
                                         cv::VideoWriter::fourcc('H','2','6','4'), 20.0, 
                                         cv::Size(framesOfRGB[0].cols, framesOfRGB[0].rows));
        if (!video_writer_rgb.isOpened()) {
            ROS_ERROR("Failed to open rgb video writer.");
        } else {
            for (const auto& frame : framesOfRGB) {
                video_writer_rgb.write(frame);
            }
            video_writer_rgb.release();
            is_rgb_success = true;
            ROS_INFO("rgb video write successfully.");
        }
    }

    if (!framesOfdepth.empty()) {
        cv::VideoWriter video_writer_depth(base_save_video_path + depthVideoFileName, 
                                           cv::VideoWriter::fourcc('H','2','6','4'), 20.0, 
                                           cv::Size(framesOfdepth[0].cols, framesOfdepth[0].rows));
        if (!video_writer_depth.isOpened()) {
            ROS_ERROR("Failed to open depth video writer.");
        } else {
            for (const auto& frame : framesOfdepth) {
                video_writer_depth.write(frame);
            }
            video_writer_depth.release();
            is_depth_success = true;
        }
    }

    return is_rgb_success && is_depth_success;
}

// 服务回调函数
bool recordVideoCallback(streaming_media::RecordVideo::Request  &req, streaming_media::RecordVideo::Response &res) {
    if (req.record) {
        recording = true;
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        sub_rgb = it.subscribe("/usb_cam/image_raw", 10, boost::bind(videoCaptureCallback, _1, "rgb"));
        sub_depth = it.subscribe("/camera/depth/image_raw", 10, boost::bind(videoCaptureCallback, _1, "depth"));

        ros::Time start_time = ros::Time::now();
        while (ros::ok() && recording) {
            ros::spinOnce(); // take a frame
            if ((ros::Time::now() - start_time) > ros::Duration(10.0)) { // 30 minutes
                ROS_INFO("30 minutes recording limit reached, stop recording.");
                isTimeOut = true;
                recording = false;
                break;
            }
        }


        bool success = write_video_frame_into_file(isTimeOut);
        if (success) {
            res.result = base_save_video_path;
        } else {
            res.result = "false";
        }
    }

    isTimeOut = false;
    return true;
}

void stopRecordingCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        recording = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "record_video_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/record_video", recordVideoCallback);
    ros::Subscriber stop_sub = nh.subscribe("/record_video_stop", 1, stopRecordingCallback); // 停止录制话题

    ROS_INFO("Ready to record videos.");

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

