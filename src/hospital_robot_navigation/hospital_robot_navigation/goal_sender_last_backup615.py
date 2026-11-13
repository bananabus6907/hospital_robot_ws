#!/usr/bin/env python3
"""
Goal Sender 노드 (경유점 순차 도착 버전)
- POI ID를 받으면 현재 위치 기준 경로 테이블 조회
- 경로의 각 점(경유지/목적지)을 navigate_to_pose로 '하나씩 정확히 도착'하며 진행
- 경유지 yaw는 다음 지점 향하도록 atan2로 1회 계산
- 방 yaw는 destinations.yaml의 저장값(방문 보는 방향) 사용

위치 추적 (하이브리드):
  - 초기: current_id = 'start'
  - 각 구간 SUCCEEDED 시 그 점으로 current_id 갱신
  - 실패(ABORTED): 멈추고 amcl_pose로 재동기화 후 대기 (자동 재출발 X)

토픽:
  구독: /navigate_to_poi (std_msgs/String)
        /cancel_navigation (std_msgs/String)
        /amcl_pose (PoseWithCovarianceStamped)
  발행: /nav_state (std_msgs/String)
"""
import os
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
import yaml


STATE_IDLE = 'IDLE'
STATE_NAVIGATING = 'NAVIGATING'
STATE_ARRIVED = 'ARRIVED'
STATE_ERROR = 'ERROR'
STATE_CANCELLED = 'CANCELLED'
STATE_ESTOP = 'ESTOP'

START_POSE = {'x': -0.022, 'y': 0.254}

ROUTE_TABLE = {
    ('start', '301'): ['center1', '301'],
    ('start', '302'): ['center1', '302'],
    ('start', '303'): ['center2', '303'],
    ('301', '302'):   ['center1', '302'],
    ('301', '303'):   ['center1', 'center2', '303'],
    ('302', '303'):   ['center1', 'center2', '303'],
    ('302', '301'):   ['center1', '301'],
    ('303', '301'):   ['center2', 'center1', '301'],
    ('303', '302'):   ['center2', 'center1', '302'],
}


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('poi_file', os.path.expanduser(
            '~/hospital_robot_ws/src/hospital_robot_poi/poi/destinations.yaml'
        ))
        self.poi_file = self.get_parameter('poi_file').value
        self.destinations = self._load_poi()

        # 상태
        self.nav_state = STATE_IDLE
        self.current_id = 'start'
        self.current_goal_id = ''      # 최종 목적지
        self.current_goal_name = ''
        self._goal_handle = None
        self._latest_amcl = None

        # 비상정지
        self._estop = False            # True면 ESTOP 상태
        self._estop_saved_path = None  # 비상정지 직전 경로 (해제 시 그대로 이어감)
        self._estop_saved_index = 0
        self._estop_saved_goal = ''
        self._estop_saved_goal_name = ''

        # 순차 실행용
        self._segment_path = []        # 현재 경로 [center1, 301] 등
        self._segment_index = 0        # 지금 향하는 점의 인덱스

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group,
        )

        self.sub_goal = self.create_subscription(
            String, '/navigate_to_poi', self.goal_request_callback, 10)
        self.sub_cancel = self.create_subscription(
            String, '/cancel_navigation', self.cancel_callback, 10)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.sub_estop = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)

        # 비상정지 시 완전 정지용 (모터 구독 토픽 /cmd_vel_hc로 0 1회 발행)
        self.pub_stop = self.create_publisher(Twist, '/cmd_vel_hc', 10)

        self.pub_state = self.create_publisher(String, '/nav_state', 10)
        self.state_timer = self.create_timer(0.5, self.publish_state)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Goal Sender (경유점 순차 도착) 시작')
        self.get_logger().info(f'등록된 POI: {len(self.destinations)}개')
        self.get_logger().info(f'현재 위치: {self.current_id}')
        self.get_logger().info('  ros2 topic pub /navigate_to_poi std_msgs/String "data: 301" --once')
        self.get_logger().info('=' * 50)
        self._print_poi()

    # ---------- POI 로딩 ----------
    def _load_poi(self):
        if not os.path.exists(self.poi_file):
            self.get_logger().error(f'POI 파일 없음: {self.poi_file}')
            return {}
        with open(self.poi_file, 'r') as f:
            data = yaml.safe_load(f)
        if not data or 'destinations' not in data:
            self.get_logger().error('잘못된 POI 파일 형식')
            return {}
        return {d['id']: d for d in data['destinations']}

    def _print_poi(self):
        if not self.destinations:
            return
        self.get_logger().info('── 등록된 목적지 ──')
        for poi_id, d in self.destinations.items():
            self.get_logger().info(
                f"  [{poi_id}] {d.get('name', poi_id)} "
                f"({d['x']:.2f}, {d['y']:.2f}) type={d.get('type', '?')}")

    # ---------- amcl ----------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self._latest_amcl = msg.pose.pose

    def _resync_from_amcl(self):
        if self._latest_amcl is None:
            self.get_logger().warn('재동기화 실패: amcl_pose 이력 없음. 위치 유지.')
            return
        px = self._latest_amcl.position.x
        py = self._latest_amcl.position.y
        candidates = {'start': START_POSE}
        for pid, d in self.destinations.items():
            candidates[pid] = {'x': d['x'], 'y': d['y']}
        nearest_id, nearest_dist = None, float('inf')
        for pid, p in candidates.items():
            dist = math.hypot(px - p['x'], py - p['y'])
            if dist < nearest_dist:
                nearest_dist, nearest_id = dist, pid
        self.current_id = nearest_id
        self.get_logger().warn(
            f'🔄 재동기화: 현재 위치 ≈ [{nearest_id}] '
            f'(amcl {px:.2f},{py:.2f} / {nearest_dist:.2f}m)')

    # ---------- 경로 ----------
    def _build_path(self, target_id):
        key = (self.current_id, target_id)
        if key in ROUTE_TABLE:
            return list(ROUTE_TABLE[key])
        self.get_logger().warn(
            f'경로 테이블에 ({self.current_id}→{target_id}) 없음. 직접 이동.')
        return [target_id]

    def _make_goal(self, poi_id, next_id):
        """poi_id로 가는 NavigateToPose.Goal 생성. next_id가 있으면 경유지 yaw 계산."""
        poi = self.destinations[poi_id]
        ptype = poi.get('type', 'room')

        if ptype == 'waypoint' and next_id is not None:
            nxt = self.destinations[next_id]
            yaw = math.atan2(nxt['y'] - poi['y'], nxt['x'] - poi['x'])
        else:
            yaw = float(poi.get('yaw', 0.0))

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(poi['x'])
        ps.pose.position.y = float(poi['y'])
        ps.pose.position.z = 0.0
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = ps
        return goal

    # ---------- 목적지 요청 ----------
    def goal_request_callback(self, msg: String):
        target_id = msg.data.strip()

        # 비상정지 중에는 모든 목적지 명령 무시
        if self._estop:
            self.get_logger().warn('🛑 비상정지 상태 — 목적지 명령 무시 (해제 후 가능)')
            return

        if target_id not in self.destinations:
            self.get_logger().error(f'❌ POI "{target_id}" 없음')
            self.nav_state = STATE_ERROR
            return
        if target_id == self.current_id:
            self.get_logger().info(f'이미 [{target_id}]에 있음.')
            return

        # 주행 중이면 기존 취소
        if self.nav_state == STATE_NAVIGATING and self._goal_handle is not None:
            self.get_logger().info('⚠️ 기존 목표 취소 후 새 목표')
            self._goal_handle.cancel_goal_async()

        self._segment_path = self._build_path(target_id)
        self._segment_index = 0
        self.current_goal_id = target_id
        self.current_goal_name = self.destinations[target_id].get('name', target_id)

        self.get_logger().info(
            f'🚀 [{self.current_id}] → [{target_id}] 경로: '
            f'{" → ".join(self._segment_path)}')

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 서버 연결 실패')
            self.nav_state = STATE_ERROR
            return

        self.nav_state = STATE_NAVIGATING
        self._send_next_segment()

    def _send_next_segment(self):
        """경로의 현재 인덱스 점으로 navigate_to_pose 전송"""
        idx = self._segment_index
        path = self._segment_path
        poi_id = path[idx]
        next_id = path[idx + 1] if idx + 1 < len(path) else None

        goal = self._make_goal(poi_id, next_id)

        seg_kind = '경유지' if next_id is not None and \
            self.destinations[poi_id].get('type') == 'waypoint' else '목적지'
        self.get_logger().info(
            f'  ▶ 구간 {idx + 1}/{len(path)}: [{poi_id}] ({seg_kind})로 이동')

        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2가 목표 거절')
            self.nav_state = STATE_ERROR
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        remaining = getattr(fb, 'distance_remaining', 0.0)
        cur_target = self._segment_path[self._segment_index]
        self.get_logger().info(
            f'📍 [{cur_target}] 이동중 (남은거리 {remaining:.2f}m)',
            throttle_duration_sec=2.0)

    def result_callback(self, future):
        status = future.result().status
        self._goal_handle = None

        if status == 4:  # SUCCEEDED
            arrived_id = self._segment_path[self._segment_index]
            self.current_id = arrived_id          # 구간 도착 → 위치 갱신
            self.get_logger().info(f'  ✅ [{arrived_id}] 도착')

            # 다음 구간 있으면 진행
            if self._segment_index + 1 < len(self._segment_path):
                self._segment_index += 1
                self._send_next_segment()
            else:
                # 최종 목적지 도착
                self.nav_state = STATE_ARRIVED
                self.get_logger().info(
                    f'🎉 최종 도착! [{self.current_id}] {self.current_goal_name}')

        elif status == 5:  # CANCELED
            self.nav_state = STATE_CANCELLED
            self.get_logger().info('⛔ 주행 취소됨')
            self._resync_from_amcl()
        else:  # ABORTED 등
            self.nav_state = STATE_ERROR
            self.get_logger().error(
                f'❌ 주행 실패 (status {status}). 정지 후 재동기화.')
            self._resync_from_amcl()

    def cancel_callback(self, msg: String):
        if self._goal_handle is not None:
            self.get_logger().info('⛔ 주행 취소 요청')
            self._goal_handle.cancel_goal_async()
        else:
            self.get_logger().info('취소할 목표 없음')

    def estop_callback(self, msg: Bool):
        """비상정지 토글. True=비상정지 진입, False=해제.
        해제 시 경로를 새로 계획하지 않고, 멈추기 전 경로(_segment_path)를
        그대로 이어서 주행한다 (안 지난 경유점 건너뛰지 않음)."""
        want_estop = bool(msg.data)

        if want_estop and not self._estop:
            # ── 비상정지 진입 ──
            self._estop = True
            # 해제 후 '같은 경로 이어가기'용으로 현재 경로 상태 통째로 저장
            if self.nav_state == STATE_NAVIGATING and self._segment_path:
                self._estop_saved_path = list(self._segment_path)
                self._estop_saved_index = self._segment_index
                self._estop_saved_goal = self.current_goal_id
                self._estop_saved_goal_name = self.current_goal_name
            else:
                self._estop_saved_path = None
            # 1) nav2 목표 취소
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            # 2) 완전 정지: 0 속도 1회 발행 (잔여 명령/관성 차단)
            self.pub_stop.publish(Twist())
            self.nav_state = STATE_ESTOP
            self.get_logger().warn('🛑 비상정지! 목표 취소 + 정지. (해제 전까지 명령 무시)')

        elif not want_estop and self._estop:
            # ── 비상정지 해제 ──
            self._estop = False
            self.get_logger().info('✅ 비상정지 해제')
            if self._estop_saved_path:
                # 멈추기 전 경로/인덱스를 그대로 복원해 같은 지점부터 이어감
                self._segment_path = self._estop_saved_path
                self._segment_index = self._estop_saved_index
                self.current_goal_id = self._estop_saved_goal
                self.current_goal_name = self._estop_saved_goal_name
                self._estop_saved_path = None
                if not self.nav_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error('❌ Nav2 서버 연결 실패')
                    self.nav_state = STATE_ERROR
                    return
                cur_seg = self._segment_path[self._segment_index]
                self.get_logger().info(
                    f'  ↻ 기존 경로 이어서 재출발: [{cur_seg}]부터 '
                    f'(경로 {" → ".join(self._segment_path)})')
                self.nav_state = STATE_NAVIGATING
                self._send_next_segment()
            else:
                self.nav_state = STATE_IDLE

    def publish_state(self):
        m = String()
        cur_seg = ''
        if self.nav_state == STATE_NAVIGATING and self._segment_path:
            cur_seg = self._segment_path[self._segment_index]
        m.data = (
            f'{{"state": "{self.nav_state}", '
            f'"current_id": "{self.current_id}", '
            f'"goal_id": "{self.current_goal_id}", '
            f'"goal_name": "{self.current_goal_name}", '
            f'"segment": "{cur_seg}"}}')
        self.pub_state.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Goal Sender 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()