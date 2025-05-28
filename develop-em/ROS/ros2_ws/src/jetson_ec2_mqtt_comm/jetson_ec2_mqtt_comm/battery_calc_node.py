import json
import time
from std_msgs.msg import String, Bool
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class BatteryCalcNode(Node):
    def __init__(self):
        # 파라미터 & 상태 변수 초기화
        super().__init__('battery_calc_node')
        self.Q_nom_map = {
            'AAA':  1000.0,
            'AA':   2400.0,
            'C':    8000.0,
            }
        # 내부 저항 매핑 (Ω) — 실제 측정값으로 교체하세요
        self.R_internal_map = {
            'AAA': 0.20,
            'AA':  0.15,
            'C':   0.10,
        }
        self.reset_all()

        # QoS 설정: depth=1, reliable 전송 보장
        qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )

        # 퍼블리셔
        self.soc_pub   = self.create_publisher(String, '/battery/discharge', 10)
        self.evt_pub   = self.create_publisher(Bool,   '/battery/discharge_complete', 10)
        # 구독자
        self.create_subscription(String, '/inspection/appearance', self.cb_app, 10) # 배터리 종류를 받아온다
        self.create_subscription(String, '/arduino/uno/raw',       self.cb_discharge, qos_profile=qos) # 무부하전압, 로드전류를 0.5초마다 받아온다.

    def reset_all(self):
        """배터리 타입 변경·방전 완료 시 내부 상태 전부 초기화"""
        self.batt_type    = None       # 최근 수신된 배터리 종류
        self.Q_nom        = None       # 현재 셀의 정격용량
        self.soc_init     = None       # 초기 SoC (%) ,단순 선형 근가법으로 계산산
        self.Q_used_mAh   = 0.0        # 누적 사용 전하 (mAh)
        self.last_time    = None       # Coulomb 카운팅 마지막 시각
        self.ocv_flag     = False      # OCV로 soc_init 계산 완료 여부
        self.discharge_threshold = 5.0 # 방전 완료 컷오프 SoC (%)

    def map_ocv_to_soc(self, ocv_v: float) -> float:
        """단순 선형 근사: 0.90V→0%, 1.50V→100%"""
        soc = (ocv_v - 0.90) / (1.50 - 0.90) * 100.0
        return max(0.0, min(100.0, soc))
    
    def cb_app(self, msg: String):
        """외관 검사 결과를 받아 배터리 타입을 설정하거나, 불량 시 초기화."""
        try:
            raw = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"APPEAR JSON 파싱 실패: {msg.data}")
            return

        # 불량 배터리면 바로 초기화
        if raw.get('abnormal') is True:
            self.get_logger().info("APPEAR: 불량 배터리 감지 → 내부 상태 초기화")
            self.reset_all()
            return

        # 정상 배터리면 class 인덱스로 타입 결정
        if raw.get('abnormal') is False:
            class_map = {0: 'AAA', 1: 'AA', 2: 'C', 3: 'D'}
            batt = class_map.get(raw.get('class'), None)
            if batt in self.Q_nom_map:
                # 타입 변경 시에만 초기화
                if batt != self.batt_type:
                    self.get_logger().info(f"APPEAR: 새 배터리 타입 → {batt}")
                    self.batt_type = batt
                    self.Q_nom     = self.Q_nom_map[batt]
                    # OCV 보정·카운팅을 처음부터 다시
                    self.soc_init   = None
                    self.Q_used_mAh = 0.0
                    self.last_time  = None
                    self.ocv_flag   = False
            else:
                self.get_logger().warn(f"APPEAR: 알 수 없는 배터리 타입 인덱스: {raw.get('class')}")
        else:
            # abnormal 이 None 이나 다른 타입이면 무시
            return

    def cb_discharge(self, msg: String):
        """0.5초마다 들어오는 raw 데이터로
           1) OCV 보정(once) → soc_init 계산
           2) Coulomb 카운팅 → soc 계산
           3) 컷오프 시 초기화
        """
        try:
            raw = json.loads(msg.data)
        except ValueError as e:
            self.get_logger().warn(f"DISCHRG JSON 파싱 실패: {msg.data}")
            return

        # 배터리 타입이 설정되지 않으면 아무 계산도 하지 않음
        if not self.batt_type:
            return

        # --- 1) 무부하시 OCV 보정 타이밍 (first_connection) ---
        # 서버(아두이노) 쪽에서 무부하 전압을 줄 때 first_connection=true 로 표시해 준다
        if raw.get('first_connection', False):
            # bus 전압(V)과 전류(mA) 읽기
            V_bus = float(raw.get('bus_voltage', 0.0))
            I_mA  = float(raw.get('current',     0.0))
            I_A   = I_mA / 1000.0
                     

            # 내부 저항(Ω) 가져오기 (기본 0.1Ω)
            R_int = self.R_internal_map.get(self.batt_type, 0.1)

            # 무부하 전압 근사
            ocv_v = V_bus + I_A * R_int

            self.soc_init   = self.map_ocv_to_soc(ocv_v)
            self.last_time  = time.time()
            self.Q_used_mAh = 0.0
            self.ocv_flag   = True
            
            # 상태 전이 변수도 이때 초기화
            self.get_logger().info(
                f"[OCV] 보정: V_bus={V_bus:.2f}V, I={I_A:.3f}A, R={R_int:.3f}Ω → "
                f"OCV={ocv_v:.2f}V → SoC_init={self.soc_init:.1f}%"
            )
            return

        # --- 2) 이후 부하 구간: soc_init 준비 안 됐으면 무시 ---
        if not self.ocv_flag:
            return

        # 필수 필드 검사
        if raw.get('load_voltage') is None or raw.get('current') is None:
            return

        # 부하시 전압·전류 읽기
        try:
            current_mA = float(raw['current'])
        except (TypeError, ValueError):
            return
        now        = time.time()

        # Coulomb 적산
        dt_h = (now - self.last_time) / 3600.0
        self.last_time   = now
        self.Q_used_mAh += current_mA * dt_h

        # SoC 계산
        soc = self.soc_init - (self.Q_used_mAh / self.Q_nom) * 100.0
        soc = max(0.0, min(100.0, soc))

        # SoC 퍼블리시
        out = {
            'SoC':         round(soc, 2),
            'batteryType': self.batt_type,
            'circuitType':  raw.get('circuit_type')
        }
        self.soc_pub.publish(String(data=json.dumps(out)))
        self.get_logger().info(f"[SoC] {out}")

        # 컷오프 이벤트
        if soc <= self.discharge_threshold:
            self.evt_pub.publish(Bool(data=True))
            self.get_logger().info(f"[EVENT] Discharge complete (SoC={soc:.1f}%)")
            # 다음 배터리 기다리도록 상태 완전 초기화
            self.reset_all()
            return

        
def main(args=None):
    rclpy.init(args=args)
    node = BatteryCalcNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
