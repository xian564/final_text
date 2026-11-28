#!/usr/bin/env python3
"""按 yaw 角速度分档统计预测误差。

用法:
  python3 scripts/eval_prediction_by_rate.py /tmp/rot_eval.bag /ft/track
  python3 scripts/eval_prediction_by_rate.py /tmp/rot_eval.bag /ft/track_rot /ft/detection 30
参数:
  bag, track_topic, det_topic=/ft/detection, rate_thres=30 (deg/s)
"""
import sys, math, rosbag
from ft_msgs.msg import Detections, Track


def yaw(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def pos(p):
    return (p.x, p.y, p.z)


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main():
    bag_path = sys.argv[1]
    track_topic = sys.argv[2]
    det_topic = sys.argv[3] if len(sys.argv) > 3 else '/ft/detection'
    rate_thres = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

    bag = rosbag.Bag(bag_path)
    dets = []      # (t, pos, yaw)
    tracks = []    # (t, tracking, cur, pred)
    for topic, msg, _ in bag.read_messages(topics=[det_topic, track_topic]):
        t = msg.header.stamp.to_sec()
        if topic == det_topic:
            if msg.detections:
                d = msg.detections[0]
                dets.append((t, pos(d.pose.position), yaw(d.pose.orientation)))
        else:
            tracks.append((t, msg.tracking, pos(msg.position), pos(msg.position_predict)))
    bag.close()
    dets.sort(key=lambda x: x[0])

    def yaw_rate(t):
        prev = None
        for tt, _, yy in dets:
            if tt > t:
                break
            prev = (tt, yy)
        if prev is None:
            return 0.0
        for tt, _, yy in dets:
            if tt > prev[0]:
                dt = tt - prev[0]
                if dt <= 0:
                    return 0.0
                dy = math.atan2(math.sin(yy - prev[1]), math.cos(yy - prev[1]))
                return abs(math.degrees(dy) / dt)
        return 0.0

    hi, lo = [], []
    for t, trk, cur, pred in tracks:
        if not trk:
            continue
        cand = [d for d in dets if t < d[0] <= t + 0.25]
        if not cand:
            continue
        actual = min(cand, key=lambda d: dist(d[1], cur))[1]
        e = dist(pred, actual)
        (hi if yaw_rate(t) > rate_thres else lo).append(e)

    def stat(a):
        if not a:
            return "n=0"
        a.sort()
        return "n=%d mean=%.3f m median=%.3f m" % (
            len(a), sum(a) / len(a), a[len(a) // 2])

    print(track_topic, "yaw_rate >", rate_thres, "deg/s:", stat(hi))
    print(track_topic, "yaw_rate <=", rate_thres, "deg/s:", stat(lo))


if __name__ == '__main__':
    main()
