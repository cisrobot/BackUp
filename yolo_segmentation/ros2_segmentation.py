#!/usr/bin/env python3
import os
import cv2
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterAlreadyDeclaredException

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge

from ultralytics import YOLO
import torch
from sensor_msgs_py import point_cloud2 as pc2  # PointCloud2 helper

# TF 변환용
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from rclpy.qos import qos_profile_sensor_data


# ========================== 토픽 ==========================
COLOR_IMG_TOPIC   = '/camera/d455_camera/color/image_raw'
DEPTH_IMG_TOPIC   = '/camera/d455_camera/aligned_depth_to_color/image_raw'
COLOR_INFO_TOPIC  = '/camera/d455_camera/color/camera_info'
ALIGNED_INFO_TOPIC= '/camera/d455_camera/aligned_depth_to_color/camera_info'
# =========================================================

# ========================== 파라미터(설정값) ==========================
VIDEO_SAVE_DIR = '/media/marin/4cca4ad9-422b-4ad3-b582-3f9c402dd434/home/omo/videos/seg'
MODEL_PATH = '/home/marin/marine/src/yolo_segmentation/models/best.pt'

CONF_THRESHOLD = 0.35
IMG_SIZE = 768
RETINA_MASKS = True

SIDEWALK_CLASS_ID = 0
SIDEWALK_CLASS_NAME_CANDIDATES = ["sidewalk", "pavement", "인도"]

ALPHA = 0.5
MASK_COLOR = (0, 0, 255)

# === 포인트클라우드/깊이 관련 ===
SIDEWALK_DEPTH_M = 10.0     # 인도 = free(멀리)
PC_MAX_RANGE_M   = 6.0      # 전방 유효 범위
PC_DOWNSAMPLE    = 1
PC_DOWNSAMPLE_NO_DET = 4
EDGE_VERTICAL_THICKNESS = 10 # 경계선 세로 두께(연속성↑)

# 극성 안전 스위치(기본: 인도=free / 비인도=장애물)
INVERT_SIDEWALK_MASK = False

# 시각화/저장
SHOW_WINDOW = True
WRITE_VIDEO = True
# =====================================================================


class YoloSegmentationNode(Node):
    """YOLO-SEG → modify_depth + 좌/우 경계만 PointCloud2(설정된 target_frame로 TF 변환)."""

    def __init__(self):
        super().__init__('ros2_segmentation_node')

        # 이 노드도 시뮬레이션 시간 사용(rosbag과 동기). 이미 선언돼 있으면 예외 없이 통과
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass
        p = self.get_parameter_or(
            'use_sim_time',
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        )
        if bool(p.value):
            self.get_logger().info("use_sim_time: True (clock=/clock)")

        # ===== target_frame 파라미터화 (기본값 'odom') =====
        try:
            self.declare_parameter('target_frame', 'odom')
        except ParameterAlreadyDeclaredException:
            pass
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.get_logger().info(f"PointCloud will be published in frame: {self.target_frame}")

        # QoS (카메라 스트리밍에 맞춤)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        cloud_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20   # Nav2/RViz 측 MessageFilter 드롭 완화용 여유
        )
        # 구독
        self.create_subscription(Image, COLOR_IMG_TOPIC, self.image_callback, qos_profile=best_effort_qos)
        self.create_subscription(Image, DEPTH_IMG_TOPIC, self.depth_callback, qos_profile=best_effort_qos)
        self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self.caminfo_color_cb, 10)
        self.create_subscription(CameraInfo, ALIGNED_INFO_TOPIC, self.caminfo_aligned_cb, 10)
        self.create_subscription(NavSatFix, '/scooter_pixel_bev', self.pixel_callback, 10)

        # 퍼블리셔
        self.mod_depth_pub = self.create_publisher(Image, '/modify_depth', best_effort_qos)
        # Nav2 / RViz 드롭 방지를 위해 RELIABLE + 넉넉한 큐
        self.cloud_pub = self.create_publisher(PointCloud2, '/modify_cloud', cloud_qos)
        self.result_pub    = self.create_publisher(Int8, '/scooter_on_sidewalk', 10)
        self.edge_mask_pub = self.create_publisher(Image, '/edge_mask', best_effort_qos)

        # 상태
        self.bridge = CvBridge()
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.model  = YOLO(MODEL_PATH, task="segment")

        # 추론 가속
        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True
        try:
            self.model.fuse()
        except Exception:
            pass

        # 최신 프레임/마스크/카메라 파라미터
        self.latest_depth_msg = None
        self.latest_depth_np  = None  # float32 [m]
        self.latest_mask_original = None  # 255=sidewalk

        # 카메라 내부 파라미터 (CameraInfo로 갱신)
        self.fx = 910.0; self.fy = 910.0
        self.cx = 640.0; self.cy = 360.0
        self.cam_w = None; self.cam_h = None

        self.video_writer = None
        self.last_yolo_time = self.get_clock().now()
        self.uv_cache = {}

        # TF 버퍼/리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("YOLO Segmentation Node 시작 (PointCloud → TF 변환 후 target_frame 기준 퍼블리시)")

        # GPU 워밍업
        try:
            dummy = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
            with torch.inference_mode():
                _ = self.model.predict(dummy, device=self.device, imgsz=IMG_SIZE,
                                       half=(self.device!='cpu'), conf=CONF_THRESHOLD,
                                       retina_masks=RETINA_MASKS, verbose=False)
        except Exception:
            pass

    # -------------------- CameraInfo 콜백 --------------------
    def caminfo_color_cb(self, msg: CameraInfo):
        self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
        self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
        self.cam_w, self.cam_h = msg.width, msg.height

    def caminfo_aligned_cb(self, msg: CameraInfo):
        self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
        self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
        self.cam_w, self.cam_h = msg.width, msg.height

    # -------------------- Depth 콜백 --------------------
    def depth_callback(self, msg: Image):
        try:
            enc = msg.encoding
            if enc in ('16UC1', 'mono16'):
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  # uint16(mm)
                depth_m = depth_raw.astype(np.float32) / 1000.0
            else:
                depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1').astype(np.float32)
            np.nan_to_num(depth_m, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
            depth_m[depth_m < 0.0] = 0.0

            self.latest_depth_np = depth_m
            self.latest_depth_msg = msg
        except Exception as e:
            self.get_logger().warn(f"depth 변환 실패: {e}")

    # -------------------- 이미지 콜백 --------------------
    def image_callback(self, msg: Image):
        # 최소 5Hz
        now = self.get_clock().now()
        if (now - self.last_yolo_time).nanoseconds < 200_000_000:
            return
        self.last_yolo_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 오류: {e}")
            return

        # 비디오 라이터
        if WRITE_VIDEO and self.video_writer is None:
            now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            os.makedirs(VIDEO_SAVE_DIR, exist_ok=True)
            path = os.path.join(VIDEO_SAVE_DIR, f'segmentation_{now_str}.mp4')
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            h, w = frame.shape[:2]
            self.video_writer = cv2.VideoWriter(path, fourcc, 20, (w, h))
            self.get_logger().info(f"저장 경로: {path}")

        # YOLO 세그 추론
        with torch.inference_mode():
            results = self.model.predict(
                frame, device=self.device, conf=CONF_THRESHOLD, imgsz=IMG_SIZE,
                half=(self.device != 'cpu'), retina_masks=RETINA_MASKS, verbose=False
            )
        if not results:
            return

        result = results[0]
        h, w = frame.shape[:2]

        # ---- 사이드워크 클래스 ID 자동 해석 ----
        sidewalk_id = self._resolve_sidewalk_id(result)
        binary_mask_sidewalk = np.zeros((h, w), dtype=np.uint8)

        if result.masks is not None and result.boxes is not None and len(result.masks.data) > 0:
            masks_np = result.masks.data.float().cpu().numpy()  # [N, Hm, Wm]
            cls = result.boxes.cls.cpu().numpy().astype(int)     # [N]
            sel = np.where(cls == sidewalk_id)[0]
            if len(sel) > 0:
                sel_masks = masks_np[sel]                     # [K, Hm, Wm]
                max_mask = np.max(sel_masks, axis=0)          # [Hm, Wm]
                mask_resized = cv2.resize((max_mask >= 0.5).astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
                binary_mask_sidewalk = (mask_resized * 255).astype(np.uint8)

        # ---- 정확도↑ 후처리 ----
        if np.any(binary_mask_sidewalk):
            binary_mask_sidewalk = self._refine_sidewalk_mask(binary_mask_sidewalk)

        # 시각화(원본 위에 인도 마스크)
        if SHOW_WINDOW or WRITE_VIDEO:
            overlay = np.zeros_like(frame)
            overlay[binary_mask_sidewalk == 255] = MASK_COLOR
            vis = cv2.addWeighted(frame, 1 - ALPHA, overlay, ALPHA, 0)
            if SHOW_WINDOW:
                cv2.imshow("Segmentation (Original View)", vis)
            if WRITE_VIDEO and self.video_writer is not None:
                self.video_writer.write(vis)
            if SHOW_WINDOW:
                cv2.waitKey(1)

        # 최신 마스크 캐시(픽셀 질의용)
        self.latest_mask_original = binary_mask_sidewalk if np.any(binary_mask_sidewalk) else None

        # -------------------- modify_depth + 좌/우 경계 포인트 --------------------
        if self.latest_depth_np is not None:
            self._publish_modify_depth_and_cloud(binary_mask_sidewalk)

    # ---- 클래스 ID 자동 해석 ----
    def _resolve_sidewalk_id(self, result):
        names = {}
        try:
            names = result.names
        except AttributeError:
            try:
                names = self.model.model.names
            except Exception:
                names = {}
        if isinstance(names, dict) and len(names) > 0:
            lname_map = {k: str(v).strip().lower() for k, v in names.items()}
            for wanted in SIDEWALK_CLASS_NAME_CANDIDATES:
                target = wanted.lower()
                for k, v in lname_map.items():
                    if v == target:
                        return int(k)
        return int(SIDEWALK_CLASS_ID)

    # ---- 마스크 후처리(정확도 개선) ----
    def _refine_sidewalk_mask(self, mask255: np.ndarray) -> np.ndarray:
        mask = (mask255 > 127).astype(np.uint8) * 255
        kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel7, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel5, iterations=1)

        h, w = mask.shape
        num, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 127).astype(np.uint8), connectivity=8)
        if num <= 1:
            return mask

        bottom_band = int(h * 0.2)
        candidates = []
        for i in range(1, num):
            y, height = stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_HEIGHT]
            y2 = y + height
            if y2 >= h - bottom_band:
                area = stats[i, cv2.CC_STAT_AREA]
                candidates.append((area, i))
        if not candidates:
            for i in range(1, num):
                candidates.append((stats[i, cv2.CC_STAT_AREA], i))
        _, keep_idx = max(candidates)
        refined = (labels == keep_idx).astype(np.uint8) * 255
        return refined

    # ---- 좌/우 경계만 추출 ----
    def _extract_lr_edges(self, mask255: np.ndarray) -> np.ndarray:
        """가로 이웃만 비교해서 좌우(수직) 경계만 추출. (위/아래 경계 제거)"""
        m = (mask255 > 127)
        m_left = np.zeros_like(m);  m_left[:, 1:] = m[:, :-1]
        m_right = np.zeros_like(m); m_right[:, :-1] = m[:, 1:]
        left_edge = m & (~m_left)
        right_edge = m & (~m_right)
        edges = left_edge | right_edge  # 1-px 경계

        if EDGE_VERTICAL_THICKNESS > 1:
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (1, EDGE_VERTICAL_THICKNESS))
            edges = cv2.dilate(edges.astype(np.uint8) * 255, k, iterations=1) > 127
        return (edges.astype(np.uint8) * 255)

    # -------------------- modify_depth / modify_cloud --------------------
    def _publish_modify_depth_and_cloud(self, sidewalk_mask_255: np.ndarray):
        """
        - /modify_depth : 인도(255)=10m, 비인도=원본 depth
        - /modify_cloud : 좌/우 경계 픽셀만 → optical 프레임에서 점 생성 후 TF로 target_frame으로 변환
        - /edge_mask    : 경계=255 디버그용
        """
        depth_msg = self.latest_depth_msg
        depth = self.latest_depth_np
        if depth_msg is None or depth is None:
            return

        # depth 크기에 맞춤
        if (sidewalk_mask_255.shape[0] != depth.shape[0]) or (sidewalk_mask_255.shape[1] != depth.shape[1]):
            sidewalk_mask_255 = cv2.resize(sidewalk_mask_255, (depth.shape[1], depth.shape[0]),
                                           interpolation=cv2.INTER_NEAREST)

        sidewalk = (sidewalk_mask_255 == 255)
        if INVERT_SIDEWALK_MASK:
            sidewalk = ~sidewalk

        # 최신 스탬프(시뮬레이션 시간)
        now_msg = self.get_clock().now().to_msg()

        # --- modify_depth: 비인도=원본, 인도=10m ---
        mod_depth = depth.copy()
        np.nan_to_num(mod_depth, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        mod_depth[mod_depth < 0.0] = 0.0
        mod_depth[sidewalk] = SIDEWALK_DEPTH_M  # 인도=멀리

        md_msg = self.bridge.cv2_to_imgmsg(mod_depth, encoding='32FC1')
        md_msg.header = depth_msg.header
        md_msg.header.stamp = now_msg
        self.mod_depth_pub.publish(md_msg)

        # --- 좌/우 경계 마스크 생성(디버그 퍼블리시) ---
        edge_mask_255 = self._extract_lr_edges((sidewalk.astype(np.uint8) * 255))
        edge_msg = self.bridge.cv2_to_imgmsg(edge_mask_255, encoding='mono8')
        edge_msg.header = depth_msg.header
        edge_msg.header.stamp = now_msg
        self.edge_mask_pub.publish(edge_msg)

        # ----- PointCloud: 경계 픽셀만, 깊이는 "원본 depth" -----
        has_sidewalk = np.any(sidewalk)
        stride = PC_DOWNSAMPLE if has_sidewalk else max(PC_DOWNSAMPLE, PC_DOWNSAMPLE_NO_DET)

        h, w = depth.shape
        key = (h, w, stride)
        if key in self.uv_cache:
            U, V = self.uv_cache[key]
        else:
            u = np.arange(0, w, stride, dtype=np.float32)
            v = np.arange(0, h, stride, dtype=np.float32)
            U, V = np.meshgrid(u, v)
            self.uv_cache[key] = (U, V)

        edge_ds = edge_mask_255[::stride, ::stride] > 127
        z_ds = depth[::stride, ::stride]  # 원본 depth 사용!

        valid = edge_ds & (z_ds > 0.0) & (z_ds <= PC_MAX_RANGE_M)
        if not np.any(valid):
            # 빈 클라우드를 target_frame 기준으로 (스탬프는 now)
            empty = pc2.create_cloud_xyz32(depth_msg.header, [])
            empty.header.frame_id = self.target_frame
            empty.header.stamp = now_msg
            self.cloud_pub.publish(empty)
            return

        zv = z_ds[valid]
        uv = U[valid]
        vv = V[valid]

        # ---- CameraInfo 해상도 ↔ depth 해상도 불일치 보정 (스케일링) ----
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        if self.cam_w is not None and self.cam_h is not None and (self.cam_w != w or self.cam_h != h):
            sx = float(w) / float(self.cam_w)
            sy = float(h) / float(self.cam_h)
            fx *= sx; fy *= sy; cx *= sx; cy *= sy

        # ---- (1) optical 프레임에서 투영 (x:우, y:아래, z:전방)
        x_opt = (uv - cx) * zv / fx
        y_opt = (vv - cy) * zv / fy
        z_opt = zv
        pts_opt = np.column_stack((x_opt, y_opt, z_opt)).astype(np.float32)

        # ---- (2) PointCloud2를 "optical 프레임"으로 생성
        optical_frame = depth_msg.header.frame_id  # 보통 ..._optical_frame
        cloud_opt = pc2.create_cloud_xyz32(depth_msg.header, pts_opt.tolist())
        cloud_opt.header.frame_id = optical_frame
        cloud_opt.header.stamp = now_msg  # 생성 시점에 now로 맞춤

        # ---- (3) TF로 target_frame으로 변환해서 퍼블리시
        try:
            # 최신 TF로 변환 (Time() == 0 → latest transform)
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,                 # to
                optical_frame,                     # from
                Time(),                            # 최신 TF (stamp=0)
                timeout=Duration(seconds=0.5)      # TF 가끔 늦게 올라오는 상황 대비
            )
            cloud_out = do_transform_cloud(cloud_opt, tf)
            cloud_out.header.frame_id = self.target_frame
            cloud_out.header.stamp = now_msg       # 최신 스탬프로 교체
            self.cloud_pub.publish(cloud_out)
        except TransformException as ex:
            # 변환 실패 시: target_frame으로 강제 퍼블리시(스탬프 now) — RViz/Costmap 드롭 방지
            self.get_logger().warn(f"TF transform 실패: {ex}. frame={self.target_frame}로 스탬프(now)와 함께 퍼블리시합니다.")
            cloud_opt.header.frame_id = self.target_frame
            cloud_opt.header.stamp = now_msg
            self.cloud_pub.publish(cloud_opt)

    # -------------------- 디버그/실험: 픽셀 질의 --------------------
    def pixel_callback(self, msg: NavSatFix):
        if self.latest_mask_original is None:
            return
        u = int(msg.longitude)
        v = int(msg.latitude)
        if 0 <= v < self.latest_mask_original.shape[0] and 0 <= u < self.latest_mask_original.shape[1]:
            is_on_sidewalk = int(self.latest_mask_original[v, u] == 255)
        else:
            is_on_sidewalk = 0
        self.result_pub.publish(Int8(data=is_on_sidewalk))

    # -------------------- 종료 처리 --------------------
    def destroy_node(self):
        if self.video_writer:
            try:
                self.video_writer.release()
            except Exception:
                pass
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
