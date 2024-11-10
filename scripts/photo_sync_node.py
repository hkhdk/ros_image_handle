#!/usr/bin/env python
import rospy
import re
import subprocess
from streaming_media.msg import SyncResult 
from std_msgs.msg import String
from datetime import datetime

sync_pub = rospy.Publisher("/photo_sync_result", SyncResult, queue_size=1)

def parse_rsync_output(line, msg):
    pattern = re.compile(r"(\d+),(\d+),(\d+),(\d+)\s+(\d+)%\s+(\d+\.\d+)MB/s\s+(\d+:\d+:\d+)")
    match = pattern.search(line)
    if match:
        msg.transferred_size = int(match.group(1))
        msg.total_size = int(match.group(2))
        msg.percent_complete = float(match.group(3)) / 100.0
        msg.speed = float(match.group(4)) / 1024.0 / 1024.0  # Convert MB/s to bytes/s
        msg.eta_seconds = (
            int(match.group(6)[0:2]) * 3600 +
            int(match.group(6)[3:5]) * 60 +
            int(match.group(6)[6:8])
        )

def rsync_callback():
    start_time = rospy.Time.now()
    try:
        process = subprocess.Popen(
            "rsync --bwlimit=500 -avzP /path/to/local/file username@remote_host:/path/to/remote/directory",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT
        )

        while True:
            line = process.stdout.readline()
            if not line:
                break
            line = line.decode('utf-8').rstrip()
            rospy.loginfo(line)
            msg = SyncResult()
            msg.file_name = "file_name"
            parse_rsync_output(line, msg)
            sync_pub.publish(msg)

        end_time = rospy.Time.now()
        elapsed = (end_time - start_time).to_sec()
        if elapsed < 1.0:  # small file
            msg = SyncResult()
            msg.percent_complete = 100.0
            sync_pub.publish(msg)

    except Exception as e:
        rospy.logerr("Failed to execute rsync command: {}".format(e))
    finally:
        process.stdout.close()
        process.wait()

if __name__ == "__main__":
    rospy.init_node("photo_sync_node", anonymous=True)
    rsync_callback()
    rospy.spin()
