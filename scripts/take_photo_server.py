#!/usr/bin/env python
import rospy
import numpy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from streaming_media.srv import *

base_save_photo_path = "/home/images/"
is_rgb_success = False
is_depth_success = False

def save_image(image, filename, path):
    full_path = base_save_photo_path + path + filename

    if path == 'depth/':
        if not cv2.imwrite(full_path, image.astype(np.uint16)):
            rospy.logerr("Failed to save image.")
            return False
        return True
    
    elif path == 'rgb/':
        if not cv2.imwrite(full_path, image):
            rospy.logerr("Failed to save image.")
            return False
        return True

def take_photo_callback(req):
    global is_rgb_success
    global is_depth_success

    is_rgb_success = is_depth_success = False

    if req.record:  # take photo (a frame)
        sub_rgb = rospy.Subscriber("/usb_cam/image_raw", Image, callback_rgb)
        sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)

        rospy.sleep(1)

        sub_rgb.unregister()
        sub_depth.unregister()

    return TakePhotoResponse(base_save_video_path if is_rgb_success and is_depth_success else "false")



def callback_rgb(msg):
    global is_rgb_success
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        rgb_path = "rgb/"
        if save_image(cv_image, "rgb_image.jpg", rgb_path):
            rospy.loginfo("RGB image saved to: %s" % rgb_path)
            is_rgb_success = True
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def callback_depth(msg):
    global is_depth_success
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "mono16")
        depth_path = "depth/"
        if save_image(cv_image, "depth_image.png", depth_path):
            rospy.loginfo("Depth image saved to: %s" % depth_path)
            is_depth_success = True
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == "__main__":
    rospy.init_node("take_photo_server")
    service = rospy.Service("/take_photo", TakePhoto, take_photo_callback)
    rospy.loginfo("Ready to take photos.")
    rospy.spin()
