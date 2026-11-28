#!/usr/bin/env python3
# 单路调试图截屏: rosrun final_text capture_debug.py _topic:=/ft/debug/image _out:=/tmp/shot _n:=5
import os, rospy, cv_bridge
from sensor_msgs.msg import Image

out = rospy.get_param('~out', '/tmp/shot')
n = int(rospy.get_param('~n', 5))
os.makedirs(out, exist_ok=True)
bridge = cv_bridge.CvBridge()
count = [0]

def cb(msg):
    if count[0] >= n:
        return
    try:
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except Exception as e:
        rospy.logwarn('cv_bridge: %s', e); return
    path = os.path.join(out, 'shot_%03d.png' % count[0])
    __import__('cv2').imwrite(path, img)
    rospy.loginfo('saved %s', path)
    count[0] += 1

rospy.init_node('capture_debug')
rospy.Subscriber(rospy.get_param('~topic', '/ft/debug/image'), Image, cb)
rospy.spin()
