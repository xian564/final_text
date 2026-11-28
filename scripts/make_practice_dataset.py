#!/usr/bin/env python3
# 从 bag 截帧 + 用现有 ONNX 模型生成伪标签，做练习数据集
# 用法: python3 make_practice_dataset.py --bag bag/red_4.bag --out practice --n 20
import os, sys, argparse, cv2, numpy as np, onnxruntime as ort
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def letterbox(img, W=640, H=480):
    h, w = img.shape[:2]
    s = min(W/w, H/h)
    nw, nh = int(round(w*s)), int(round(h*s))
    px, py = (W-nw)//2, (H-nh)//2
    can = np.full((H, W, 3), 124, np.uint8)
    can[py:py+nh, px:px+nw] = cv2.resize(img, (nw, nh))
    return can, s, px, py

def order(pts):
    s = [p[0]+p[1] for p in pts]; d = [p[1]-p[0] for p in pts]
    idx = [int(np.argmin(s)), int(np.argmin(d)), int(np.argmax(s)), int(np.argmax(d))]
    if len(set(idx)) < 4:
        c = np.mean(pts, axis=0)
        idx = sorted(range(4), key=lambda i: np.arctan2(pts[i][1]-c[1], pts[i][0]-c[0]))
    return [pts[i] for i in idx]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--out', default='practice')
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--model', default='/root/catkin_ws/bag/armor_allfp32.onnx')
    args = ap.parse_args()

    os.makedirs(args.out+'/images/train', exist_ok=True)
    os.makedirs(args.out+'/images/val', exist_ok=True)
    os.makedirs(args.out+'/labels/train', exist_ok=True)
    os.makedirs(args.out+'/labels/val', exist_ok=True)

    sess = ort.InferenceSession(args.model, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0].name
    bridge = CvBridge()
    bag = rosbag.Bag(args.bag)
    saved = 0
    for topic, msg, _ in bag.read_messages(topics=['/hk_camera/image_raw/compressed']):
        if saved >= args.n: break
        img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        can, s, px, py = letterbox(img)
        blob = can[:, :, ::-1].astype(np.float32)/255.0
        blob = np.transpose(blob, (2,0,1))[None]
        out = sess.run(None, {inp: blob})[0][0]
        lines = []
        for i in range(out.shape[1]):
            c = int(np.argmax(out[0:3, i])); cs = float(out[c, i])
            if cs < 0.5: continue
            n = int(np.argmax(out[4:13, i])); ns = float(out[4+n, i])
            if ns < 0.5: continue
            pts = []
            for k in range(4):
                x = (out[13+2*k, i]-px)/s; y = (out[14+2*k, i]-py)/s
                pts.append([x, y])
            pts = order(pts)
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
            x1, x2, y1, y2 = min(xs), max(xs), min(ys), max(ys)
            h, w = img.shape[:2]
            cx, cy = (x1+x2)/2/w, (y1+y2)/2/h
            bw, bh = (x2-x1)/w, (y2-y1)/h
            kp = ' '.join(f'{p[0]/w:.6f} {p[1]/h:.6f}' for p in pts)
            lines.append(f'0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f} {kp}')
        if not lines: continue
        split = 'val' if saved % 5 == 0 else 'train'
        cv2.imwrite(f'{args.out}/images/{split}/frame_{saved:03d}.jpg', img)
        open(f'{args.out}/labels/{split}/frame_{saved:03d}.txt','w').write('\n'.join(lines)+'\n')
        saved += 1
    bag.close()
    print('saved', saved)

if __name__ == '__main__':
    main()
