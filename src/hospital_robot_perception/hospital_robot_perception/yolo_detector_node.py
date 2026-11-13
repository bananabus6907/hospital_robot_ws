#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # 파라미터들
        self.declare_parameter('model_path', '/home/team9/yolo/best.pt')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('process_hz', 3.0)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('conf_threshold', 0.70)
        self.declare_parameter('min_box_height', 100)
        self.declare_parameter('min_box_width', 30)
        self.declare_parameter('aspect_ratio_min', 1.3)
        self.declare_parameter('box_height_stop', 200)
        self.declare_parameter('center_band', 0.30)
        self.declare_parameter('confirm_frames', 2)

        gp = self.get_parameter
        self.model_path = gp('model_path').value
        self.image_topic = gp('image_topic').value
        self.period = 1.0 / float(gp('process_hz').value)
        self.imgsz = int(gp('imgsz').value)
        self.conf = float(gp('conf_threshold').value)
        self.min_h = int(gp('min_box_height').value)
        self.min_w = int(gp('min_box_width').value)
        self.aspect_min = float(gp('aspect_ratio_min').value)
        self.box_h_stop = int(gp('box_height_stop').value)
        self.center_band = float(gp('center_band').value)
        self.confirm_frames = int(gp('confirm_frames').value)

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)
        self.device = 'cuda' if __import__('torch').cuda.is_available() else 'cpu'
        
        self.pub_detected = self.create_publisher(Bool, '/perception/human_detected', 10)
        self.pub_height = self.create_publisher(Float32, '/perception/human_box_height', 10)
        self.pub_debug_img = self.create_publisher(Image, '/perception/debug_image', 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 1)

        self._last_t = 0.0
        self._hit_streak = 0

    def on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_t < self.period: return
        self._last_t = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h_img, w_img = frame.shape[:2]

        results = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, verbose=False, device=self.device)

        valid = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            w, h = x2 - x1, y2 - y1
            ratio = h / w if w > 0 else 0
            # 비율 필터링 추가 (1.2 ~ 4.0 사이의 세로형 박스만 통과)
            if ratio < 1.2 or ratio > 4.0: continue
            if h < self.min_h or w < self.min_w: continue
            valid.append({'cx': (x1 + x2) / 2.0, 'h': h})

        in_stop_zone = False
        nearest_h = 0.0
        if valid:
            biggest = max(valid, key=lambda b: b['h'])
            nearest_h = biggest['h']
            in_center = abs(biggest['cx'] - w_img / 2.0) <= self.center_band * w_img
            close = biggest['h'] >= self.box_h_stop
            in_stop_zone = in_center and close

        self._hit_streak = self._hit_streak + 1 if in_stop_zone else 0
        detected = self._hit_streak >= self.confirm_frames

        # 시각화 추가
        annotated_frame = results[0].plot()
        cv2.line(annotated_frame, (int(w_img/2 - self.center_band*w_img), 0), (int(w_img/2 - self.center_band*w_img), h_img), (255, 0, 0), 2)
        cv2.line(annotated_frame, (int(w_img/2 + self.center_band*w_img), 0), (int(w_img/2 + self.center_band*w_img), h_img), (255, 0, 0), 2)
        self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))

        m = Bool(); m.data = bool(detected); self.pub_detected.publish(m)
        mh = Float32(); mh.data = float(nearest_h); self.pub_height.publish(mh)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()