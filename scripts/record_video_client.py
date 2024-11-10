#!/usr/bin/env python
import rospy
from streaming_media.srv import *

def main():
    rospy.init_node('record_video_client')

    try:
        rospy.wait_for_service("/record_video")
        client = rospy.ServiceProxy('/record_video', RecordVideo)

    
        res = client(True)
        rospy.loginfo("Video capture: %s" % res.result)

    except rospy.ServiceException, e:
       rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    main()
