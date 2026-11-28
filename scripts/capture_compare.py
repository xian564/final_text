#!/usr/bin/env python3
# 同帧对比截图：传统 /ft/debug/image 与 NN /ft/debug/image_nn
# 用法: rosrun final_text capture_compare.py _out:=/tmp/compare _n:=5
import os, rospy, cv_bridge, message_filters
from sensor_msgs.msg import Image

class Capture:
    def __init__(self):
        self.out = rospy.get_param('~out', '/tmp/compare')
        self.n = int(rospy.get_param('~n', 5))
        os.makedirs(self.out, exist_ok=True)
        self.bridge = cv_bridge.CvBridge()
        self.count = 0
        a = message_filters.Subscriber('/ft/debug/image', Image)
        b = message_filters.Subscriber('/ft/debug/image_nn', Image)
        sync = message_filters.ApproximateTimeSynchronizer([a, b], 10, 0.2)
        sync.registerCallback(self.cb)
        rospy.loginfo('capture to %s, n=%d', self.out, self.n)

    def cb(self, ta, tb):
        if self.count >= self.n:
            return
        try:
            ia = self.bridge.imgmsg_to_cv2(ta, 'bgr8')
            ib = self.bridge.imgmsg_to_cv2(tb, 'bgr8')
        except Exception as e:
            rospy.logwarn('cv_bridge: %s', e)
            return
        h = max(ia.shape[0], ib.shape[0])
        canvas = 255 * __import__('numpy').ones((h, ia.shape[1] + ib.shape[1] + 10, 3), 'uint8')
        canvas[:ia.shape[0], :ia.shape[1]] = ia
        canvas[:ib.shape[0], ia.shape[1] + 10:] = ib
        path = os.path.join(self.out, 'compare_%03d.png' % self.count)
        __import__('cv2').imwrite(path, canvas)
        rospy.loginfo('saved %s', path)
        self.count += 1

if __name__ == '__main__':
    rospy.init_node('capture_compare')
    Capture()
    rospy.spin()
