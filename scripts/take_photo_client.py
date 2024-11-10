#!/usr/bin/env python
import rospy
from streaming_media.srv import *

def main():
    rospy.init_node('take_photo_client')

    try:
        rospy.wait_for_service("/take_photo")
        client = rospy.ServiceProxy('/take_photo', TakePhoto)

        res = client(True)
        rospy.loginfo("Photo taken: %s" % res.result)

    except rospy.ServiceException, e:
       rospy.loginfo("Service call failed: %s" % e)


if __name__ == '__main__':
    main()
