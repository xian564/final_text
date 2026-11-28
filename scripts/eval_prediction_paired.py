#!/usr/bin/env python3
"""同一时间戳上成对比较匀速/旋转的预测误差。

用法:
  python3 scripts/eval_prediction_paired.py /tmp/nn_rot.bag /ft/detection_nn 30
"""
import sys, math, rosbag
from ft_msgs.msg import Detections, Track


def yaw(q):
    s = 2 * (q.w * q.z + q.x * q.y)
    c = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def pos(p):
    return (p.x, p.y, p.z)


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main():
    bag_path = sys.argv[1]
    det_topic = sys.argv[2] if len(sys.argv) > 2 else '/ft/detection'
    rate_thres = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0

    bag = rosbag.Bag(bag_path)
    dets, A, B = [], [], []
    for topic, m, _ in bag.read_messages(
            topics=[det_topic, '/ft/track', '/ft/track_rot']):
        t = m.header.stamp.to_sec()
        if topic == det_topic:
            if m.detections:
                d = m.detections[0]
                dets.append((t, pos(d.pose.position), yaw(d.pose.orientation)))
        elif topic == '/ft/track':
            A.append((t, m.tracking, pos(m.position), pos(m.position_predict)))
        else:
            B.append((t, m.tracking, pos(m.position), pos(m.position_predict)))
    bag.close()
    dets.sort(key=lambda x: x[0])

    def rate(t):
        prev = None
        for tt, _, yy in dets:
            if tt > t:
                break
            prev = (tt, yy)
        if not prev:
            return 0.0
        for tt, _, yy in dets:
            if tt > prev[0]:
                dt = tt - prev[0]
                if dt <= 0:
                    return 0.0
                d = math.atan2(math.sin(yy - prev[1]), math.cos(yy - prev[1]))
                return abs(math.degrees(d) / dt)
        return 0.0

    pairs = []
    for t, ok, cur, pred in A:
        if not ok:
            continue
        cand = [x for x in B if abs(x[0] - t) < 0.01 and x[1]]
        if not cand:
            continue
        nxt = [d for d in dets if t < d[0] <= t + 0.25]
        if not nxt:
            continue
        act = min(nxt, key=lambda d: dist(d[1], cur))[1]
        pairs.append((rate(t), dist(pred, act), dist(cand[0][3], act)))

    hi = [p for p in pairs if p[0] > rate_thres]
    lo = [p for p in pairs if p[0] <= rate_thres]

    def st(a, i):
        v = sorted(x[i] for x in a)
        if not v:
            return "n=0"
        return "n=%d mean=%.3f median=%.3f" % (
            len(v), sum(v) / len(v), v[len(v) // 2])

    print("paired total", len(pairs))
    print("yaw_rate >", rate_thres, " const:", st(hi, 1), " rot:", st(hi, 2))
    print("yaw_rate <=", rate_thres, " const:", st(lo, 1), " rot:", st(lo, 2))


if __name__ == '__main__':
    main()
