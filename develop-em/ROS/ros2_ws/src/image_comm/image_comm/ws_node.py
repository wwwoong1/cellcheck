#!/usr/bin/env python3
"""
ws_node.py
이 노드는 Jetson Nano에서 ROS2와 asyncio를 결합해 WebSocket 통신을 처리합니다.
주요 동작:
 1. '/camera/raw' 토픽으로부터 sensor_msgs/Image 메시지를 구독
 2. 수신된 프레임을 JPEG로 인코딩 후 Base64로 직렬화해 내부 버퍼에 저장
 3. 버퍼가 지정된 크기(batch_size)에 도달하거나 타임아웃(batch_timeout)이 지나면
    버퍼 내용을 JSON 페이로드로 WebSocket 서버에 전송
 4. 서버로부터 수신한 JSON 분석 결과를 '/inspection/appearance' 토픽에 퍼블리시
 5. 전송 성공, 버퍼 오버플로우, 연결 오류 등의 상태를 '/analysis/health' 토픽에 퍼블리시
 6. ROS2 메인 스레드와 asyncio 백그라운드 스레드를 락(Lock)으로 안전하게 동기화
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import asyncio
import websockets
from websockets import exceptions as ws_exc
import base64
import json
import threading
from threading import Lock
import traceback

class WSNode(Node):
    def __init__(self):
        super().__init__('ws_node')
        # --- 1) 파라미터 선언 및 읽기 ---
        self.declare_parameter('server_uri','')
        self.declare_parameter('batch_size',    3)
        self.declare_parameter('max_buffer',   10)
        self.declare_parameter('batch_timeout', 1.0)

        raw_uri = self.get_parameter('server_uri').value or ''
        self.server_uri = raw_uri.strip()
        #self.server_uri    = self.get_parameter('server_uri').value.strip()
        self.get_logger().info(f"Loaded server_uri: {self.server_uri!r}")
        self.get_logger().info(f"[DEBUG] server_uri={self.get_parameter('server_uri').value!r}")
        self.send_batch    = self.get_parameter('batch_size').value
        self.max_buffer    = self.get_parameter('max_buffer').value
        self.batch_timeout = self.get_parameter('batch_timeout').value

        # --- 2) ROS2 통신 설정 ---
        # 2-1) 카메라 이미지 구독
        self.sub = self.create_subscription(
            Image, '/camera/raw', self.image_callback, 10)
        # 2-2) 분석 결과 퍼블리시
        self.pub = self.create_publisher(
            String, '/inspection/appearance', 10)
        # 2-3) 헬스 상태 퍼블리시(QoS: reliable)
        health_qos = QoSProfile(depth=10)
        health_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.health_pub = self.create_publisher(
            String, '/analysis/health', health_qos)

        # --- 3) 내부 상태 및 락 초기화 ---
        self.br        = CvBridge()          # ROS↔OpenCV 브리지
        self.buffer    = []                 # 직렬화된 이미지 버퍼
        self.connected = False              # WebSocket 연결 상태
        self.ws        = None               # WebSocket 객체
        self.lock      = Lock()             # 스레드 동기화 락
        self.last_send = self.get_clock().now()  # 마지막 전송 시각
        
        # ① 결과 누적용 버퍼
        self.results = []
        
        # --- 4) asyncio 이벤트 루프를 별도 스레드에서 실행 ---
        self.loop   = asyncio.new_event_loop()
        self.thread = threading.Thread(
            target=self.loop.run_forever, daemon=True)
        self.thread.start()
        # 백그라운드에서 연결/수신 루프 시작
        asyncio.run_coroutine_threadsafe(self.ws_connect(), self.loop)

    def image_callback(self, msg: Image):
        """
        이미지 콜백:
         1) ROS Image → OpenCV 배열 → JPEG 압축 → Base64 인코딩
         2) 내부 버퍼에 추가(최대 max_buffer 유지, overflow 시 헬스 퍼블리시)
         3) batch_size 또는 batch_timeout 조건 만족 시
            락 하에서 상태 복사 → 락 해제 후 WebSocket 전송 및 헬스 퍼블리시
        """
        # 1) 이미지 직렬화
        if not msg.header.frame_id.startswith('segment_'):
            return
        frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
        _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        b64 = base64.b64encode(buf).decode('ascii')

        now = self.get_clock().now()
        elapsed = (now - self.last_send).nanoseconds * 1e-9

        # 2) 락 안에서 버퍼 및 상태 처리
        with self.lock:
            # 버퍼에 추가
            self.buffer.append(b64)
            # overflow 시 가장 오래된 프레임 제거하고 경고 퍼블리시
            if len(self.buffer) > self.max_buffer:
                self.buffer.pop(0)
                self.health_pub.publish(String(data=json.dumps({
                    'level': 'warning',
                    'issue': 'buffer_overflow',
                    'buffer_size': len(self.buffer)
                })))

            # 전송 조건 검사
            if not (len(self.buffer) >= self.send_batch or elapsed >= self.batch_timeout):
                return  # 아직 전송 조건 미충족

            # 전송을 위한 상태 복사
            ws        = self.ws
            connected = self.connected
            batch     = list(self.buffer)
            self.buffer.clear()
            self.last_send = now

        # 3) 락 해제 후 WebSocket 전송 및 헬스 퍼블리시
        if ws and connected:
            for idx, img_b64 in enumerate(batch, start=1):
                #단일 이미지 페이로드
                payload = json.dumps({
                    "image": img_b64
                })
                # 비동기 전송 스케줄
                asyncio.run_coroutine_threadsafe(ws.send(payload), self.loop)
                self.get_logger().info(f"> 1장 전송 완료 ({idx}/{len(batch)})")
                # 전송 헬스 퍼블리시 (원하시는 필드만 남기셔도 됩니다)
                self.health_pub.publish(String(data=json.dumps({
                    'level': 'info',
                    'issue': 'sent',
                    'image_index': idx,
                    'elapsed': elapsed
                })))

            # for 문 다 돌면 최종 send 완료 로그
            self.get_logger().info(f"> 총 {len(batch)}장 순차 전송 완료")
            
        else:
            # 연결이 끊긴 경우 전송 중단 헬스 퍼블리시
            self.health_pub.publish(String(data=json.dumps({
                'level': 'error',
                'issue': 'not_connected'
            })))
            self.get_logger().warning("WebSocket 미연결 상태에서 전송 시도")

    async def ws_connect(self):
        self.get_logger().info(f"Attempting WebSocket connection to {self.server_uri!r}")
        while True:
            try:
                # ── 1) 서버 연결 ──
                ws_new = await websockets.connect(self.server_uri)
                with self.lock:
                    self.ws = ws_new
                    self.connected = True
                self.get_logger().info("WS 연결 성공")

                # ── 2) 메시지 수신 루프 ──
                while True:
                    try:
                        resp = await ws_new.recv()
                    except ws_exc.ConnectionClosedOK:
                        self.get_logger().info("WS: 서버 정상 종료")
                        # 내부 루프만 빠져나가서 재접속 대기로
                        break
                    except ws_exc.ConnectionClosedError as e:
                        self.get_logger().warning(f"WS 비정상 종료: {e}")
                        break

                    # ── 2-1) JSON 파싱 & 검증 ──
                    try:
                        obj = json.loads(resp)
                        # 1) 딕셔너리 여부 체크
                        if not isinstance(obj, dict):
                            raise ValueError(f"Expected dict, got {type(obj).__name__}")
                        # 2) 필수 키 존재 여부 체크
                        if 'class' not in obj or 'bbox' not in obj:
                            raise KeyError("Missing 'class' or 'bbox'")
                        # 3) bbox 형태 검사 (리스트 & 길이 4)
                        if not (isinstance(obj['bbox'], list) and len(obj['bbox']) == 4):
                            raise ValueError(f"Invalid bbox format: {obj['bbox']!r}")
                        # 4) abnormal 기본값
                        obj.setdefault("abnormal", False)
                    except Exception as e:
                        self.get_logger().error(f"파싱 오류: {e}")
                        # fallback 퍼블리시
                        fallback = {"abnormal": True, "error": str(e)}
                        self.pub.publish(String(data=json.dumps(fallback)))
                        continue

                    # ── 2-2) 배치 & 최종 퍼블리시 ──
                    with self.lock:
                        self.results.append(obj)
                        if len(self.results) < self.send_batch:
                            continue
                        batch = list(self.results)
                        self.results.clear()

                    final = next((r for r in batch if r["abnormal"]), batch[-1])
                    self.pub.publish(String(data=json.dumps(final)))
                    self.get_logger().info(f"최종 결과 퍼블리시: {final}")

            # ── 커넥션 단계 예외 ──
            except ws_exc.InvalidURI as e:
                self.get_logger().error(f"잘못된 URI: {e}")
                break
            except ws_exc.InvalidHandshake as e:
                self.get_logger().error(f"핸드셰이크 실패: {e}")
                break
            except OSError as e:
                self.get_logger().warning(f"네트워크 오류: {e}")
                # 잠깐 대기 후 재접속
                await asyncio.sleep(5)
                continue

            # ── 의도치 않은 예외만 여기서 잡아서 루프 종료 ──
            except Exception as e:
                tb = traceback.format_exc()
                self.get_logger().error(f"WS 연결 루프 오류: {e}\n{tb}")
                break

            finally:
                # ── 연결 정리 ──
                with self.lock:
                    if self.ws:
                        await self.ws.close()
                        self.ws = None
                    self.connected = False

            # ── try/except/finally 바깥, while True 레벨에서 항상 실행 ──
            self.get_logger().info("5초 후 재연결 시도...")
            await asyncio.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    node = WSNode()
    try:
        rclpy.spin(node)  # ROS2 메인 이벤트 루프
    finally:
        # 종료 시 WebSocket 정리 및 asyncio 루프 중단
        if node.ws:
            asyncio.run_coroutine_threadsafe(node.ws.close(), node.loop)
        node.loop.call_soon_threadsafe(node.loop.stop)
        node.thread.join(timeout=1)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
