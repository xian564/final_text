#!/usr/bin/env python3
# 录 /ft/debug/image 为帧序列（10Hz），用于拼演示视频
# 用法: python3 record_debug_video.py /tmp/vid/s1 40
import os, sys, time, rospy, cv_bridge, cv2
from sensor_msgs.msg import Image

out = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
os.makedirs(out, exist_ok=True)
bridge = cv_bridge.CvBridge()
last = [0.0]; idx = [0]

def cb(msg):
    t = time.time()
    if t - last[0] < 0.1:   # 10 Hz
        return
    last[0] = t
    try:
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except Exception:
        return
    cv2.imwrite(os.path.join(out, 'f_%05d.jpg' % idx[0]), img)
    idx[0] += 1

rospy.init_node('rec_video')
rospy.Subscriber('/ft/debug/image', Image, cb)
end = time.time() + secs
while time.time() < end and not rospy.is_shutdown():
    time.sleep(0.05)
print('frames', idx[0])
