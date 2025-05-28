#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2

# 이미지 선명도 측정 헬퍼
def variance_of_laplacian(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()

class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        # 파라미터 선언 & 로드
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('save_dir', '~/Desktop/captured_images')
        self.fps = self.get_parameter('fps').value
        raw_dir = self.get_parameter('save_dir').value
        self.save_dir = os.path.abspath(os.path.expanduser(raw_dir))
        os.makedirs(self.save_dir, exist_ok=True)

        # 퍼블리셔 & 구독자 설정
        self.pub = self.create_publisher(Image, '/camera/raw', 10)
        self.health_sub = self.create_subscription(
            String, '/analysis/health', self.health_callback, 10
        )

        # ─── 새로 추가된 상태 변수 ───
        self.abnormal = False               # 아직 abnormal 판정 결과를 못 받음
        self.discharge_complete = True    # False면 /battery/discharge_complete True 대기


        self.block_after_discharge = None  # 3초 블록
        self.block_after_abnormal   = None  # 5초 블록

        # ────────── IR 센서 이벤트 구독 추가 ──────────
        self.ir_sub = self.create_subscription(Bool, '/sensor/ir_b', self.ir_callback, 10)

        # ─── 분류 결과(appearance) 구독 ───
        self.create_subscription(
            String,
            '/inspection/appearance',
            self.classification_callback,
            10
        )

        # ─── 방전 완료 구독 ───
        self.create_subscription(
            Bool,
            '/battery/discharge_complete',
            self.discharge_callback,
            10
        )

        # 캡처 상태 초기화
        self.capturing = False
        self.duration = 3.0
        self.segments = 3
        self.seg_duration = self.duration / self.segments
        self.best_scores = [0.0] * self.segments
        self.best_frames = [None] * self.segments
        self.start_time = None

        # 카메라 초기화 (GStreamer)
        self.br = CvBridge()
        gst_str = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1, format=NV12 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패')
            raise RuntimeError('Camera open failed')

        # fps 주기 타이머
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def ir_callback(self, msg: Bool):
        """IR 센서 트리거 → capture 시작 조건 검사."""
        if not msg.data or self.capturing:
            return
        
        now = self.get_clock().now()
        # 0) 아직 classification 안 받았으면 무시
        if self.abnormal is None:
            self.get_logger().info('Ignoring IR: waiting for first classification')
            return
    
        # 1)discharge_complete 신호 후 3초 동안 IR 무시
        if self.block_after_discharge is not None and now < self.block_after_discharge:
            self.get_logger().info('Ignoring IR: within 3s block after discharge')
            return

        # 2) abnormal=True 면 5초 블록 검사
        if self.abnormal:
            if self.block_after_abnormal and now < self.block_after_abnormal:
                self.get_logger().info('Ignoring IR: within 5s block after abnormal=True')
                return
        else:  # self.abnormal is False
            if not self.discharge_complete:
                self.get_logger().info('Ignoring IR: waiting for discharge_complete')
                return
            # discharge_complete True면 IR 허용

        # 여기를 통과하면 캡처 시작
        self.get_logger().info('IR sensor triggered → start capture')
        self.start_capture()

    def classification_callback(self, msg: String):
        """분류 결과 수신 → abnormal 값 셋업 및 블록 타임 설정."""
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warning(f'Invalid classification JSON: {msg.data}')
            return
    
        is_abnormal = data.get('abnormal', False)   

        now = self.get_clock().now()

        # 방전 블록이 남아있으면 해제 (classification 후 즉시 촬영 가능하게)
        self.block_after_discharge = None

        # 1) abnormal 상태 갱신
        self.abnormal = is_abnormal

        if is_abnormal:
            # 2) abnormal=True → 5초 블록
            self.block_after_abnormal = now + Duration(seconds=5)
            self.get_logger().info('Received abnormal=True → blocking IR for 5s')
        else:
            # 3) abnormal=False → 다음 discharge_complete 대기 상태로 전환
            self.block_after_abnormal = None
            self.get_logger().info('Received abnormal=False → waiting for discharge')

    def discharge_callback(self, msg: Bool):
        """방전 완료 신호 수신 → IR를 3초간 블록"""
        if msg.data:
            now = self.get_clock().now()
            # 3초간 IR 무시
            self.discharge_complete = True
            self.block_after_discharge = now + Duration(seconds=3)
            # 이 플래그는 더 이상 'capture 허용'용이 아니라 3초 블록용입니다
            self.get_logger().info('Received discharge_complete → blocking IR for 3s')

    def start_capture(self):
        """캡처 초기화 (기존 start_capture 로직을 여기로 옮김)."""
        self.capturing = True
        self.start_time = self.get_clock().now()
        self.best_scores = [0.0] * self.segments
        self.best_frames = [None] * self.segments
        self.discharge_complete = False

    def timer_callback(self):
        """캡처 진행 중이면 프레임 평가 및 최종 저장/발행."""
        if not self.capturing:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if elapsed > self.duration:
            self.get_logger().info('Capture complete, saving & publishing')
            for idx, frame in enumerate(self.best_frames):
                if frame is None:
                    self.get_logger().warning(f'No frame for segment {idx+1}')
                    continue
                fname = os.path.join(self.save_dir, f'best_seg{idx+1}.jpg')
                cv2.imwrite(fname, frame)
                self.get_logger().info(f'Saved {fname} (score={self.best_scores[idx]:.2f})')
                img_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = now.to_msg()
                img_msg.header.frame_id = f'segment_{idx+1}'
                self.pub.publish(img_msg)
            self.capturing = False
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warning('Failed to read frame')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        score = variance_of_laplacian(gray)
        seg_idx = min(int(elapsed // self.seg_duration), self.segments - 1)
        if score > self.best_scores[seg_idx]:
            self.best_scores[seg_idx] = score
            self.best_frames[seg_idx] = frame.copy()

    def health_callback(self, msg: String):
        """네트워크 상태에 따라 FPS 조정 혹은 캡처 중단."""
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warning('Health JSON parse error')
            return

        level = data.get('level')
        issue = data.get('issue')
        if level == 'warning' and issue == 'buffer_overflow':
            new_fps = max(1.0, self.fps / 2.0)
            if abs(new_fps - self.fps) > 1e-3:
                self.get_logger().info(f'Reducing FPS {self.fps}→{new_fps}')
                self._reset_timer(new_fps)
        elif level == 'error' and issue == 'not_connected':
            self.get_logger().info('Stopping capture due to health error')
            if self.timer:
                self.timer.cancel()
                self.timer = None

    def _reset_timer(self, new_fps):
        if self.timer:
            self.timer.cancel()
        self.fps = new_fps
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def destroy_node(self):
        if self.timer:
            self.timer.cancel()
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()