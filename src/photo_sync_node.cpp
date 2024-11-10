#include <ros/ros.h>
#include <streaming_media/SyncResult.h>  // streaming_media 包名
#include <boost/regex.hpp>
#include <iostream>
#include <cstdio>
#include <string>
#include <chrono>
#include <thread>

constexpr int BUF_SIZE = 2048;

std::chrono::steady_clock::time_point start_time;
std::chrono::steady_clock::time_point end_time;
ros::Publisher sync_pub;

void parseRsyncOutput(const std::string& line, streaming_media::SyncResult& msg) {
    boost::regex e("(\\d+),(\\d+),(\\d+),(\\d+)\\s+(\\d+)%\\s+(\\d+\\.\\d+)MB/s\\s+(\\d+:\\d+:\\d+)");
    boost::smatch match;
    if (boost::regex_search(line, match, e)) {
        msg.transferred_size = atoll(match[1].str().c_str());
        msg.total_size = atoll(match[2].str().c_str());
        msg.percent_complete = std::stof(match[3].str()) / 100.0f;
        msg.speed = std::stof(match[4].str()) / 1024.0f / 1024.0f;  // Convert MB/s to bytes/s
        msg.eta_seconds = std::stof(match[5].str().substr(0, match[5].str().size() - 3)) * 3600 +
                          std::stof(match[5].str().substr(match[5].str().size() - 2)) * 60 +
                          std::stoi(match[5].str().substr(match[5].str().size() - 1));
    }
}

void rsyncCallback(void) {
    FILE* pipe = popen("rsync --bwlimit=500 -avzP /path/to/local/file username@remote_host:/path/to/remote/directory", "r");
    if (!pipe) {
        ROS_ERROR("Failed to open pipe for rsync command.");
        return;
    }

    char buf[BUF_SIZE];
    std::string line;
    streaming_media::SyncResult msg;
    start_time = std::chrono::steady_clock::now();
    while (fgets(buf, BUF_SIZE, pipe) != NULL) {  // pipe -> buf(one line msg)
        std::string line(buf);
        parseRsyncOutput(line, msg);
        msg.file_name = "file_name";
        sync_pub.publish(msg);
    }

    end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    if (elapsed.count() < 1.0) { // small file
        msg.percent_complete = 100.0;
        sync_pub.publish(msg);
    }

    pclose(pipe);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "photo_sync_node");
    ros::NodeHandle nh;

    ros::Publisher sync_pub = nh.advertise<streaming_media::SyncResult>("/photo_sync_result", 1);
    rsyncCallback();
    // ros::Timer timer = nh.createTimer(ros::Duration(1.0), rsyncCallback);

    return 0;
}
