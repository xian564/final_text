#!/usr/bin/env python3
# 评估 position_predict 与“下一帧实际检测”的误差（不用真值）
import sys, math, rosbag
from ft_msgs.msg import Detections, Track

def pos(p): return (p.x, p.y, p.z)
def dist(a,b): return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))

def main(bag_path, track_topic, det_topic='/ft/detection'):
    bag = rosbag.Bag(bag_path)
    dets=[]; tracks=[]
    for topic,msg,_ in bag.read_messages(topics=[det_topic, track_topic]):
        if topic==det_topic:
            dets.append((msg.header.stamp.to_sec(), [pos(d.pose.position) for d in msg.detections]))
        else:
            tracks.append((msg.header.stamp.to_sec(), msg.tracking,
                           pos(msg.position), pos(msg.position_predict)))
    bag.close()
    errs=[]
    for t,trk,cur,pred in tracks:
        if not trk: continue
        nxt=[d for d in dets if t < d[0] <= t+0.25]
        if not nxt: continue
        cand=[p for p in nxt[0][1]]
        if not cand: continue
        # 用当前位置匹配下一帧的同一块板
        actual=min(cand, key=lambda p: dist(p,cur))
        errs.append(dist(pred,actual))
    if errs:
        errs.sort()
        print(track_topic, "n=",len(errs),
              "mean=%.3f m"% (sum(errs)/len(errs)),
              "median=%.3f m"% errs[len(errs)//2])
    else:
        print(track_topic,"no samples")

if __name__=='__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv)>3 else '/ft/detection')
