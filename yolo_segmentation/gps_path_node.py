#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def llh_to_ecef(lat: float, lon: float, h: float) -> Tuple[float, float, float]:
    """WGS84 (lat[rad], lon[rad], h[m]) -> ECEF (X,Y,Z) [m]."""
    a = 6378137.0  # semi-major
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    X = (N + h) * cos_lat * cos_lon
    Y = (N + h) * cos_lat * sin_lon
    Z = (N * (1.0 - e2) + h) * sin_lat
    return X, Y, Z


def ecef_to_enu(x: float, y: float, z: float,
                x0: float, y0: float, z0: float,
                lat0: float, lon0: float) -> Tuple[float, float, float]:
    """ECEF -> ENU (m) with origin at (lat0, lon0, h0)."""
    dx, dy, dz = x - x0, y - y0, z - z0
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)
    t = [
        [-sin_lon0,              cos_lon0,               0.0],
        [-sin_lat0 * cos_lon0,  -sin_lat0 * sin_lon0,   cos_lat0],
        [ cos_lat0 * cos_lon0,   cos_lat0 * sin_lon0,   sin_lat0],
    ]
    e = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz
    n = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz
    u = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz
    return e, n, u


class GpsPathNode(Node):
    def __init__(self):
        super().__init__('gps_path_node')

        # ---- Parameters ----
        self.declare_parameter('frame_id', 'odom')                  # Path 좌표계
        self.declare_parameter('path_topic', '/gps_path')          # 출력 Path 토픽
        self.declare_parameter('gps_topic', '/mavros/global_position/global')
        self.declare_parameter('min_distance_m', 0.3)              # 포인트 간 최소 간격
        self.declare_parameter('use_altitude', False)              # z 에 고도 반영 여부
        self.declare_parameter('max_points', 10000)                # Path 길이 제한
        self.declare_parameter('downsample_every', 1)              # N번째 메시지마다 기록

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.use_altitude = bool(self.get_parameter('use_altitude').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.downsample_every = int(self.get_parameter('downsample_every').value)

        # QoS: Sensor data 스타일
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos.durability = QoSDurabilityPolicy.VOLATILE

        self.sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_cb, qos)
        self.pub = self.create_publisher(Path, self.path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        # ENU 기준점(첫 유효 NavSatFix)
        self.ref_llh: Optional[Tuple[float, float, float]] = None
        self.ref_ecef: Optional[Tuple[float, float, float]] = None

        self._msg_count = 0
        self._last_xy: Optional[Tuple[float, float]] = None

        self.get_logger().info(
            f"gps_path_node started. Subscribing: {self.gps_topic} -> Publishing: {self.path_topic} (frame_id: {self.frame_id})"
        )

    def gps_cb(self, msg: NavSatFix):
        self._msg_count += 1

        # 기본 유효성 체크
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        # 다운샘플
        if self.downsample_every > 1 and (self._msg_count % self.downsample_every) != 0:
            return

        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)
        alt = msg.altitude if self.use_altitude and not math.isnan(msg.altitude) else 0.0

        # 기준점 설정
        if self.ref_llh is None:
            self.ref_llh = (lat_rad, lon_rad, alt)
            self.ref_ecef = llh_to_ecef(lat_rad, lon_rad, alt)
            self.get_logger().info(
                f"Reference set (lat, lon, alt): {msg.latitude:.8f}, {msg.longitude:.8f}, {alt:.2f} (ENU origin)"
            )

        # LLH -> ECEF -> ENU
        x, y, z = llh_to_ecef(lat_rad, lon_rad, alt)
        e, n, u = ecef_to_enu(x, y, z, *self.ref_ecef, self.ref_llh[0], self.ref_llh[1])

        # 최소 이동 거리 필터
        if self._last_xy is not None:
            dx = e - self._last_xy[0]
            dy = n - self._last_xy[1]
            if (dx*dx + dy*dy) < (self.min_distance_m * self.min_distance_m):
                return

        self._last_xy = (e, n)

        # PoseStamped 추가
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(e)
        pose.pose.position.y = float(n)
        pose.pose.position.z = float(u if self.use_altitude else 0.0)
        pose.pose.orientation.w = 1.0  # 방향정보 없으니 단위쿼터니언

        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose)

        # 길이 제한
        if len(self.path_msg.poses) > self.max_points:
            self.path_msg.poses = self.path_msg.poses[-self.max_points:]

        # 퍼블리시
        self.pub.publish(self.path_msg)

    def destroy_node(self):
        self.get_logger().info(f"Shutting down. Total poses: {len(self.path_msg.poses)}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
