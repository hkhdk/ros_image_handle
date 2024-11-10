#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from streaming_media.srv import *
from std_msgs.msg import Bool
from threading import Lock

recording = False
isTimeOut = False
base_save_video_path = "/home/videos/"
framesOfRGB = []
framesOfDepth = []

def video_capture_callback_rgb(msg):
    global recording, framesOfRGB, framesOfDepth
    if not recording:
        return
    try:
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8")
        framesOfRGB.append(cv_image)
    except cv_bridge.CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)

def video_capture_callback_depth(msg):
    global recording, framesOfRGB, framesOfDepth
    if not recording:
        return
    try:
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, "mono16")
        framesOfDepth.append(cv_image)
    except cv_bridge.CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)


def write_video_frame_into_file(isTimeOut):
    global framesOfRGB, framesOfDepth
    is_rgb_success = False
    is_depth_success = False

    rgb_video_file_name = "rgb_video.mp4" if not isTimeOut else "rgb_video_timeout.mp4"
    depth_video_file_name = "depth_video.mp4" if not isTimeOut else "depth_video_timeout.mp4"

    if framesOfRGB:
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        video_writer_rgb = cv2.VideoWriter(base_save_video_path + rgb_video_file_name, fourcc, 20.0, (framesOfRGB[0].shape[1], framesOfRGB[0].shape[0]))
        if not video_writer_rgb.isOpened():
            rospy.logerr("Failed to open rgb video writer.")
        else:
            for frame in framesOfRGB:
                video_writer_rgb.write(frame)
            video_writer_rgb.release()
            is_rgb_success = True

    if framesOfDepth:
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        video_writer_depth = cv2.VideoWriter(base_save_video_path + depth_video_file_name, fourcc, 20.0, (framesOfDepth[0].shape[1], framesOfDepth[0].shape[0]))
        if not video_writer_depth.isOpened():
            rospy.logerr("Failed to open depth video writer.")
        else:
            for frame in framesOfDepth:
                video_writer_depth.write(frame)
            video_writer_depth.release()
            is_depth_success = True

    return is_rgb_success and is_depth_success


def record_video_callback(req):
    global recording, isTimeOut, framesOfRGB, framesOfDepth
    if req.record:
        recording = True
        sub_rgb = rospy.Subscriber("/usb_cam/image_raw", Image, video_capture_callback_rgb, queue_size=24)
        sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, video_capture_callback_depth, queue_size=24)

        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and recording:
            if (rospy.Time.now() - start_time) > rospy.Duration(30, 0):  # 30 minutes
                rospy.loginfo("30 minutes recording limit reached, stop recording.")
                isTimeOut = True
                recording = False
                break

        success = write_video_frame_into_file(isTimeOut)
        sub_rgb.unregister()
        sub_depth.unregister()
        return RecordVideoResponse(base_save_video_path if success else "false")

    return RecordVideoResponse("false")

def stop_recording_callback(msg):
    global recording
    if msg.data:
        recording = False

if __name__ == "__main__":
    rospy.init_node("record_video_server")

    service = rospy.Service("/record_video", RecordVideo, record_video_callback)
    stop_sub = rospy.Subscriber("/record_video_stop", Bool, stop_recording_callback)

    rospy.loginfo("Ready to record videos.")

    rospy.spin()
