#!/usr/bin/env python3
# NN 装甲板检测节点（ONNX Runtime Python 后端）
# 模型协议：1x3x480x640，等比缩放+补边124，RGB/255；输出[1,21,6300]
import rospy, cv2, numpy as np, onnxruntime as ort
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ft_msgs.msg import Detections

class NnDetector:
    def __init__(self):
        self.model = rospy.get_param('~model_path', '')
        self.color_th = rospy.get_param('~color_thres', 0.5)
        self.num_th = rospy.get_param('~number_thres', 0.5)
        self.nms_th = rospy.get_param('~nms_thres', 0.45)
        self.W = rospy.get_param('~armor_large_w', 0.230)
        self.H = rospy.get_param('~armor_large_h', 0.060)
        self.ws = rospy.get_param('~armor_small_w', 0.135)
        self.hs = rospy.get_param('~armor_small_h', 0.060)
        self.scale_pnp = rospy.get_param('~pnp_scale', 1.0)
        self.lock_gate = rospy.get_param('~lock_gate_px', 300.0)
        self.K = None; self.D = None; self.frame = 'hk_camera_ca'
        self.br = CvBridge(); self.prev = None
        self.sess = ort.InferenceSession(self.model, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        rospy.Subscriber('/hk_camera/image_raw/compressed', CompressedImage, self.cb, queue_size=1)
        rospy.Subscriber('/hk_camera/camera_info', rospy.AnyMsg, self.info_dummy, queue_size=1)
        self.pub = rospy.Publisher('/ft/detection_nn', Detections, queue_size=1)
        self.img_pub = rospy.Publisher('/ft/debug/image_nn', Image, queue_size=1)
        rospy.loginfo('NN detector ready: %s', self.model)

    def info_dummy(self, msg):
        # 直接用默认内参（如需可换成 CameraInfo 解析）
        pass

    def letterbox(self, img):
        h, w = img.shape[:2]
        s = min(640.0 / w, 480.0 / h)
        nw, nh = int(round(w * s)), int(round(h * s))
        px, py = (640 - nw) // 2, (480 - nh) // 2
        canvas = np.full((480, 640, 3), 124, np.uint8)
        canvas[py:py+nh, px:px+nw] = cv2.resize(img, (nw, nh))
        return canvas, s, px, py

    def cb(self, msg):
        img = self.br.compressed_imgmsg_to_cv2(msg, 'bgr8')
        canvas, s, px, py = self.letterbox(img)
        blob = canvas[:, :, ::-1].astype(np.float32) / 255.0     # RGB
        blob = np.transpose(blob, (2, 0, 1))[None]
        out = self.sess.run(None, {self.inp: blob})[0][0]        # 21 x 6300
        dets = []
        for i in range(out.shape[1]):
            c = int(np.argmax(out[0:3, i])); cs = float(out[c, i])
            if cs < self.color_th: continue
            n = int(np.argmax(out[4:13, i])); ns = float(out[4+n, i])
            if ns < self.num_th: continue
            pts = []
            for k in range(4):
                x = (out[13+2*k, i] - px) / s
                y = (out[14+2*k, i] - py) / s
                pts.append([x, y])
            # 模型 TL,BL,BR,TR -> TL,TR,BR,BL
            quad = [pts[0], pts[3], pts[2], pts[1]]
            minx = min(p[0] for p in pts); maxx = max(p[0] for p in pts)
            miny = min(p[1] for p in pts); maxy = max(p[1] for p in pts)
            dets.append({'color': c, 'num': n, 'score': cs, 'quad': quad,
                         'box': [minx, miny, maxx-minx, maxy-miny]})
        if dets:
            boxes = [tuple(map(int, d['box'])) for d in dets]
            scores = [d['score'] for d in dets]
            keep = cv2.dnn.NMSBoxes(boxes, scores, self.color_th, self.nms_th)
            dets = [dets[int(k)] for k in np.array(keep).flatten()] if len(keep) else []
        # 相邻帧锁同一块
        pick = 0
        if dets and self.prev is not None:
            best = 1e18
            for k, d in enumerate(dets):
                c = np.mean(np.array(d['quad']), axis=0)
                dist = float(np.linalg.norm(c - self.prev))
                if dist < best: best, pick = dist, k
            if best > self.lock_gate: pick = 0
        arr = Detections(); arr.header = msg.header
        canvas = img.copy()
        for k, d in enumerate(dets):
            color = (0,0,255) if d['color']==1 else (255,0,0) if d['color']==0 else (128,128,128)
            q = np.array(d['quad'], np.int32)
            cv2.polylines(canvas, [q], True, color, 3 if k==pick else 1)
            cv2.putText(canvas, 'ID:%d' % d['num'], tuple(q[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if k != pick: continue
            # 简化 PnP：用默认内参
            if self.K is None:
                self.K = np.array([[1798.0,0,700.0],[0,1798.0,593.0],[0,0,1]], np.float64)
                self.D = np.zeros((1,5))
            from ft_msgs.msg import Detection
            import tf.transformations as tft
            wpx = float(np.linalg.norm(q[0]-q[1])); hpx = float(np.linalg.norm(q[0]-q[3]))
            large = wpx > 1e-3 and wpx/hpx > 3.0
            Wb = (self.W if large else self.ws)*0.5*self.scale_pnp
            Hb = (self.H if large else self.hs)*0.5*self.scale_pnp
            obj = np.array([[-Wb,-Hb,0],[Wb,-Hb,0],[Wb,Hb,0],[-Wb,Hb,0]], np.float32)
            ok, rvec, tvec = cv2.solvePnP(obj, np.array(d['quad'], np.float32), self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
            if not ok or tvec[2] < 0.3 or tvec[2] > 15: continue
            det = Detection(); det.header = msg.header
            det.id = k+1; det.is_red = (d['color']==1); det.is_large_armor = bool(large)
            det.pose.position.x = float(tvec[0]); det.pose.position.y = float(tvec[1]); det.pose.position.z = float(tvec[2])
            R,_ = cv2.Rodrigues(rvec); qq = tft.quaternion_from_matrix(np.vstack([np.hstack([R, tvec]), [0,0,0,1]]))
            det.pose.orientation.x, det.pose.orientation.y, det.pose.orientation.z, det.pose.orientation.w = map(float, qq)
            arr.detections.append(det)
            self.prev = np.mean(np.array(d['quad']), axis=0)
        if arr.detections: self.pub.publish(arr)
        self.img_pub.publish(self.br.cv2_to_imgmsg(canvas, 'bgr8'))

if __name__ == '__main__':
    rospy.init_node('nn_detector_node')
    NnDetector(); rospy.spin()
