#!/usr/bin/env python3
"""统计 /ft/detection 的 yaw 范围，判断旋转片段。

用法:
  python3 scripts/check_yaw.py /tmp/rot_det.bag
  python3 scripts/check_yaw.py /tmp/nn_rot.bag /ft/detection_nn
"""
import sys
import math
import rosbag
from ft_msgs.msg import Detections


def yaw(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(s, c))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/rot_det.bag'
    topic = sys.argv[2] if len(sys.argv) > 2 else '/ft/detection'
    bag = rosbag.Bag(path)
    ys = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        if msg.detections:
            ys.append(yaw(msg.detections[0].pose.orientation))
    bag.close()
    if not ys:
        print('no detections on', topic, 'in', path)
        return
    print('n =', len(ys))
    print('min = %.2f deg' % min(ys))
    print('max = %.2f deg' % max(ys))
    print('range = %.2f deg' % (max(ys) - min(ys)))


if __name__ == '__main__':
    main()
