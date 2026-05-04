#!/usr/bin/env python3

"""
object_contour_service_server.py

이 파일의 역할:
- RealSense 카메라에서 color/depth 프레임을 가져옵니다.
- YOLO로 물체 이름(label)과 큰 bbox를 찾습니다.
- bbox 내부에서 depth mask + color edge를 이용해 실제 물체 외곽을 더 세밀하게 잡습니다.
- cv2.minAreaRect()로 물체의 회전각(angle)을 계산합니다.
- 픽셀 좌표 + depth 값을 이용해 카메라 기준 3D 좌표를 구합니다.
- T_BASE_CAM 행렬로 카메라 좌표를 로봇 base 좌표로 바꿉니다.
- /detect_objects 서비스 응답으로 ObjectInfo[]를 반환합니다.

즉, 로봇팔 노드는 이 서비스를 호출해서
물체 이름, 위치, 회전각, 신뢰도 정보를 받아 pick/place에 사용합니다.
"""

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import pyrealsense2 as rs
import math
from collections import deque
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# 사용자가 만든 ROS2 인터페이스입니다.
# ObjectInfo.msg: 물체 하나의 정보
# DetectObjects.srv: 물체 인식 서비스 요청/응답
from dakae_interfaces.msg import ObjectInfo
from dakae_interfaces.srv import DetectObjects
from geometry_msgs.msg import Vector3

from ultralytics import YOLO


# ==========================================================
# 1. 전체 설정값
# ==========================================================

# RealSense color/depth 해상도와 FPS입니다.
# YOLO 입력 이미지는 color frame을 사용하고,
# depth frame은 해당 픽셀의 3D 위치를 구하는 데 사용합니다.
STREAM_WIDTH = 1280
STREAM_HEIGHT = 720
STREAM_FPS = 30


# depth 값 필터링 범위입니다.
# 너무 가까운 값이나 너무 먼 값은 잘못된 depth일 가능성이 있어서 제거합니다.
DEPTH_MIN = 0.1
DEPTH_MAX = 2.0

# 테이블 깊이를 추정할 때 사용할 percentile입니다.
# 화면 안의 valid depth 중 80% 지점 값을 테이블 깊이로 보고,
# 그보다 카메라에 가까운 픽셀을 물체 후보로 판단합니다.
TABLE_DEPTH_PERCENTILE = 80

# 테이블 깊이보다 이 정도 이상 가까운 픽셀을 물체로 봅니다.
OBJECT_DEPTH_MARGIN = 0.015

# depth mask 노이즈 제거용 morphology kernel 크기입니다.
MORPH_KERNEL_SIZE = 5

# 중심 픽셀 주변 depth median을 구할 때 사용하는 커널 크기입니다.
# 단일 픽셀 depth는 노이즈가 많기 때문에 주변 5x5 median을 사용합니다.
DEPTH_MEDIAN_KSIZE = 5

# depth 유효 픽셀이 너무 적으면 mask 생성을 포기합니다.
MIN_VALID_DEPTH_PIXELS = 1000


# YOLO 모델 후보 이름입니다.
# dakae_v2_best.pt가 있으면 먼저 쓰고, 없으면 best.pt를 사용합니다.
YOLO_MODEL_CANDIDATES = ("dakae_v2_best.pt", "best.pt")
YOLO_CONF_THRESH = 0.45
YOLO_IOU_THRESH = 0.45
YOLO_IMGSZ = 640


# contour / angle 계산 관련 설정입니다.
MIN_CONTOUR_AREA = 300
ANGLE_SMOOTH_N = 7
MIN_HULL_POINTS = 5
MIN_ASPECT_RATIO = 1.15
MAX_ANGLE_JUMP_DEG = 30.0

# ==========================================================
# 결과 안정화 설정
# ==========================================================
# YOLO/Depth 결과가 프레임마다 조금씩 흔들리므로,
# 같은 물체로 판단되는 결과를 최근 N프레임 동안 저장한 뒤 평균값을 사용합니다.
OBJECT_AVG_N = 10

# 같은 물체인지 판단할 때 사용하는 base 좌표 거리 기준입니다.
# 단위는 meter입니다. 0.05면 약 5cm 이내의 같은 label 물체를 같은 물체로 봅니다.
OBJECT_MATCH_DIST = 0.05

# 평균을 내기 전에 최소 몇 번 이상 관측되어야 stable하다고 볼지 결정합니다.
# 너무 작으면 흔들림이 남고, 너무 크면 처음 인식까지 시간이 걸립니다.
OBJECT_MIN_OBS = 3

# color edge를 뽑을 때 사용하는 Canny 파라미터입니다.
CANNY_LOW = 30
CANNY_HIGH = 100
EDGE_DILATE_KSIZE = 3

# YOLO bbox를 그대로 쓰면 테두리에 배경/테이블 노이즈가 섞일 수 있어서
# bbox 내부를 살짝 줄여서 ROI를 사용합니다.
ROI_INNER_PAD = 5


# 디버그 창 설정입니다.
DEBUG_WINDOW = "object_detection_debug"
PREVIEW_PERIOD_SEC = 0.1


# 디버그 화면에서 class별 bbox 색상입니다.
CLASS_COLORS = {
    "doll": (255, 100, 100),
    "box": (100, 255, 100),
    "cube": (100, 100, 255),
    "bottle": (255, 255, 0),
    "toy": (180, 120, 255),
    "doll-cube-box-bottle": (0, 255, 255),
}


# ==========================================================
# 2. YOLO 모델 경로 찾기
# ==========================================================

def get_yolo_model_path() -> str:
    """
    YOLO 모델 파일 경로를 찾는 함수입니다.

    우선 dakae_vision 패키지 share 디렉토리에서 모델을 찾고,
    실패하면 현재 파일 기준 상위 폴더에서 찾습니다.
    """

    def first_existing(base_path: Path) -> Path:
        # 후보 모델 이름을 순서대로 확인합니다.
        for name in YOLO_MODEL_CANDIDATES:
            candidate = base_path / name
            if candidate.exists():
                return candidate

        # 실제 파일이 없어도 첫 번째 후보 경로를 반환합니다.
        # 이후 YOLO()에서 파일이 없으면 에러가 발생합니다.
        return base_path / YOLO_MODEL_CANDIDATES[0]

    try:
        # ROS2 패키지 install/share/dakae_vision 경로를 찾습니다.
        package_share = Path(get_package_share_directory("dakae_vision"))
        return str(first_existing(package_share))
    except Exception:
        # 패키지 share 경로를 못 찾으면 현재 파일 위치 기준으로 찾습니다.
        return str(first_existing(Path(__file__).resolve().parents[1]))


# logger.info()가 없는 환경을 대비한 안전 로그 함수입니다.
def log_info(node: Node, message: str):
    logger = node.get_logger()
    info = getattr(logger, "info", None)
    if info is not None:
        info(message)
    else:
        logger.dinfo(message)


# ==========================================================
# 3. 서비스 서버 노드 클래스
# ==========================================================

class ObjectContourServiceServer(Node):
    """
    /detect_objects 서비스를 제공하는 ROS2 노드입니다.

    로봇팔 노드는 이 서비스에 요청을 보내고,
    이 노드는 현재 카메라 화면에서 물체를 찾아 응답합니다.
    """

    # 카메라 좌표계의 점 P_cam을 로봇 base 좌표계 P_base로 바꾸는 행렬입니다.
    # P_base = T_BASE_CAM @ P_cam
    # 이 값은 hand-eye calibration 또는 별도 캘리브레이션으로 구한 값이어야 합니다.
    T_BASE_CAM = np.array([
        [-0.99952167, -0.02565113, -0.00438063, 0.72058381],
        [-0.02557996, 0.99941226, -0.02027756, -0.00185360],
        [0.00489439, -0.02019912, -0.99960142, 0.69734538],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)

    def __init__(self):
        super().__init__("object_contour_service_server")

        # RealSense 카메라 초기화
        self._setup_realsense()

        # YOLO 모델 로드
        self._setup_yolo()

        # 각도 smoothing을 위한 히스토리입니다.
        # key 예: "cube_0", "box_1"
        self._angle_history = {}

        # 점프 필터용 이전 angle 저장소입니다.
        self._prev_angles = {}

        # 마지막 검출 결과를 저장합니다.
        self._latest_objects = []

        # 물체별 평균값을 내기 위한 히스토리입니다.
        # key 예: "cube_0", "cube_1", "box_0"
        # 값은 최근 관측된 position, angle, confidence 등을 저장하는 deque입니다.
        self._object_tracks = {}
        self._next_track_id = 0

        # 핵심: ROS2 서비스 서버 생성입니다.
        # 서비스 이름: /detect_objects
        # 서비스 타입: DetectObjects
        # 요청이 오면 self._detect_callback() 실행
        self.srv = self.create_service(
            DetectObjects,
            "detect_objects",
            self._detect_callback
        )

        # 디버그 화면용 타이머입니다.
        # 서비스 요청이 없어도 0.1초마다 화면을 업데이트합니다.
        self.preview_timer = self.create_timer(
            PREVIEW_PERIOD_SEC,
            self._preview_callback
        )

        log_info(self, "Object Detection Service ready: /detect_objects")

    # ======================================================
    # 4. RealSense / YOLO 초기화
    # ======================================================

    def _setup_realsense(self):
        """
        RealSense color/depth stream을 켜고,
        color 기준으로 depth align을 설정합니다.
        """
        self.pipeline = rs.pipeline()
        cfg = rs.config()

        # color stream 설정
        cfg.enable_stream(
            rs.stream.color,
            STREAM_WIDTH,
            STREAM_HEIGHT,
            rs.format.bgr8,
            STREAM_FPS
        )

        # depth stream 설정
        cfg.enable_stream(
            rs.stream.depth,
            STREAM_WIDTH,
            STREAM_HEIGHT,
            rs.format.z16,
            STREAM_FPS
        )

        # RealSense pipeline 시작
        profile = self.pipeline.start(cfg)

        # depth frame을 color frame 좌표계에 맞춥니다.
        # 이걸 해야 YOLO bbox 픽셀 위치에서 정확한 depth를 읽을 수 있습니다.
        self.align = rs.align(rs.stream.color)

        # 카메라 내부 파라미터입니다.
        # 픽셀 좌표를 3D 좌표로 바꿀 때 fx, fy, ppx, ppy가 필요합니다.
        intr = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()

        self.fx = intr.fx
        self.fy = intr.fy
        self.ppx = intr.ppx
        self.ppy = intr.ppy

        # depth raw 값을 meter 단위로 바꾸기 위한 scale입니다.
        self.depth_scale = (
            profile
            .get_device()
            .first_depth_sensor()
            .get_depth_scale()
        )

        log_info(self, "RealSense initialized.")

    def _setup_yolo(self):
        """
        YOLO 모델을 로드합니다.
        """
        model_path = get_yolo_model_path()
        self.yolo = YOLO(model_path)
        log_info(self, f"YOLO loaded: {model_path}")

    # ======================================================
    # 5. 서비스 콜백
    # ======================================================

    def _detect_callback(self, request, response):
        """
        로봇팔 노드가 /detect_objects 서비스를 호출하면 실행됩니다.

        request.target_labels 예시:
        - []: 전체 물체 반환
        - ["cube"]: cube만 반환
        - ["cube", "box"]: cube와 box만 반환
        """
        try:
            # 현재 카메라 프레임을 가져옵니다.
            color_img, depth_frame = self._capture_frames()

            # 실제 비전 처리입니다.
            # 결과는 dict 리스트입니다.
            objects = self._process(color_img, depth_frame)

            # 사용자가 원하는 label만 필터링합니다.
            if request.target_labels:
                objects = [
                    obj for obj in objects
                    if obj["label"] in request.target_labels
                ]

            self._latest_objects = objects

            # 프레임마다 흔들리는 raw 결과를 바로 쓰지 않고,
            # 같은 물체끼리 매칭한 뒤 최근 값들의 평균으로 안정화합니다.
            objects = self._stabilize_objects(objects)

            # dict 리스트를 ROS 메시지 ObjectInfo 리스트로 변환합니다.
            response.detected_objects = [
                self._make_msg(obj) for obj in objects
            ]

            response.success = True
            response.message = f"Detected {len(objects)} object(s)."

        except Exception as e:
            # 예외가 발생하면 서비스 응답은 실패로 보냅니다.
            self.get_logger().error(f"Detection failed: {e}")
            response.success = False
            response.message = f"Error: {e}"

        return response

    def _preview_callback(self):
        """
        디버그 화면 표시용 콜백입니다.

        서비스 요청이 없어도 현재 카메라 화면에서 검출 결과를 계속 보여줍니다.
        로봇팔 제어에는 직접 필요 없지만 튜닝할 때 유용합니다.
        """
        try:
            color_img, depth_frame = self._capture_frames()
            objects = self._process(color_img, depth_frame)
            # 디버그 화면도 안정화된 값을 기준으로 보여줍니다.
            objects = self._stabilize_objects(objects)
            self._latest_objects = objects
            self._draw_debug(color_img, objects)

        except Exception as e:
            self.get_logger().warn(f"Preview failed: {e}")

    # ======================================================
    # 6. 프레임 획득
    # ======================================================

    def _capture_frames(self):
        """
        RealSense에서 color frame과 depth frame을 가져옵니다.
        depth frame은 color frame 기준으로 align됩니다.
        """
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            raise RuntimeError("RealSense 프레임 획득 실패")

        color_img = np.asanyarray(color_frame.get_data())
        return color_img, depth_frame

    # ======================================================
    # 7. 메인 비전 처리
    # ======================================================

    def _process(self, color_img, depth_frame):
        """
        한 장의 color/depth frame에서 물체 정보를 추출합니다.

        처리 순서:
        1. depth 이미지에서 테이블 위 물체 후보 mask 생성
        2. YOLO로 물체 bbox와 label 검출
        3. 각 bbox 안에서 contour를 정밀하게 잡음
        4. minAreaRect로 회전각 계산
        5. bbox 중심 픽셀과 depth로 3D 좌표 계산
        6. 카메라 좌표를 로봇 base 좌표로 변환
        """

        # depth frame raw 데이터를 numpy 배열로 바꾸고 meter 단위로 변환합니다.
        depth_m = (
            np.asanyarray(depth_frame.get_data()).astype(np.float32)
            * self.depth_scale
        )

        # 테이블보다 가까운 픽셀을 물체 후보로 표시한 mask입니다.
        table_mask = self._build_table_mask(depth_m)

        # YOLO로 물체를 검출합니다.
        yolo_res = self.yolo.predict(
            source=color_img,
            conf=YOLO_CONF_THRESH,
            iou=YOLO_IOU_THRESH,
            imgsz=YOLO_IMGSZ,
            device="cpu",
            verbose=False,
        )[0]

        objects = []

        # YOLO가 찾은 bbox들을 하나씩 처리합니다.
        for obj_id, box in enumerate(yolo_res.boxes):
            # YOLO bbox 좌표입니다. 왼쪽 위 x1,y1 / 오른쪽 아래 x2,y2
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            # YOLO confidence와 class label입니다.
            conf = float(box.conf[0])
            label = self.yolo.names[int(box.cls[0])]

            # bbox가 화면 밖으로 나가지 않도록 보정합니다.
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(STREAM_WIDTH - 1, x2)
            y2 = min(STREAM_HEIGHT - 1, y2)

            # 잘못된 bbox는 건너뜁니다.
            if x2 <= x1 or y2 <= y1:
                continue

            # bbox 안쪽만 사용합니다.
            # YOLO bbox 경계에 테이블/배경이 섞이면 angle이 흔들리므로 조금 줄입니다.
            rx1 = min(max(x1 + ROI_INNER_PAD, 0), STREAM_WIDTH - 1)
            ry1 = min(max(y1 + ROI_INNER_PAD, 0), STREAM_HEIGHT - 1)
            rx2 = min(max(x2 - ROI_INNER_PAD, 0), STREAM_WIDTH - 1)
            ry2 = min(max(y2 - ROI_INNER_PAD, 0), STREAM_HEIGHT - 1)

            # bbox가 너무 작아서 안쪽 crop이 불가능하면 원래 bbox를 사용합니다.
            if rx2 <= rx1 or ry2 <= ry1:
                rx1, ry1, rx2, ry2 = x1, y1, x2, y2

            # depth 기반 물체 mask의 ROI입니다.
            roi_mask = table_mask[ry1:ry2, rx1:rx2]

            # color edge를 얻기 위한 color ROI입니다.
            color_roi = color_img[ry1:ry2, rx1:rx2]

            # ROI 안에서 물체 contour를 찾고 회전각을 계산합니다.
            angle_deg, box_pts, reliable = self._calc_angle(
                roi_mask=roi_mask,
                offset=(rx1, ry1),
                color_roi=color_roi
            )

            # cube는 정사각형이라 90도 대칭 때문에 angle이 튈 수 있습니다.
            # 그래서 로봇 gripper가 쓰기 편한 -45~45 범위로 바꿉니다.
            if label == "cube":
                angle_deg = self._normalize_cube_angle(angle_deg)

            # angle smoothing + jump filter 적용입니다.
            angle_deg = self._smooth_angle(
                label=label,
                obj_id=obj_id,
                angle=angle_deg,
                reliable=reliable
            )

            # 현재 코드는 위치 중심을 YOLO bbox 중심으로 잡습니다.
            # 더 정밀하게 하려면 contour 중심으로 바꿀 수 있습니다.
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # 픽셀 좌표 + depth를 카메라 좌표계 3D 점으로 변환합니다.
            p_cam = self._pixel_to_cam(cx, cy, depth_frame)
            if p_cam is None:
                continue

            # 카메라 좌표를 로봇 base 좌표로 변환합니다.
            p_base = self.T_BASE_CAM @ p_cam

            # 하나의 물체 정보를 dict로 저장합니다.
            # 나중에 _make_msg()에서 ObjectInfo 메시지로 변환됩니다.
            objects.append({
                "id": obj_id,
                "label": label,
                "conf": conf,
                "center_px": (cx, cy),
                "bbox": (x1, y1, x2 - x1, y2 - y1),
                "angle_deg": float(angle_deg),
                "reliable": bool(reliable),
                "box_pts": box_pts,
                "area": float((x2 - x1) * (y2 - y1)),
                "position_base": (
                    float(p_base[0]),
                    float(p_base[1]),
                    float(p_base[2])
                ),
            })

        return objects

    # ======================================================
    # 8. depth mask 생성
    # ======================================================

    def _build_table_mask(self, depth_m: np.ndarray) -> np.ndarray:
        """
        depth 이미지에서 테이블보다 가까운 픽셀을 물체 후보로 만듭니다.

        반환값:
        - 물체 후보: 255
        - 배경/테이블: 0
        """

        # 유효한 depth만 가져옵니다.
        valid = depth_m[
            (depth_m > DEPTH_MIN) &
            (depth_m < DEPTH_MAX)
        ]

        # 유효 depth가 너무 적으면 빈 mask 반환
        if len(valid) < MIN_VALID_DEPTH_PIXELS:
            return np.zeros(depth_m.shape, dtype=np.uint8)

        # 테이블 깊이 추정
        table_depth = float(np.percentile(valid, TABLE_DEPTH_PERCENTILE))

        # 테이블보다 카메라에 더 가까운 픽셀만 물체 후보로 설정
        raw = (
            (depth_m > DEPTH_MIN) &
            (depth_m < table_depth - OBJECT_DEPTH_MARGIN)
        ).astype(np.uint8) * 255

        # 작은 노이즈 제거와 구멍 메우기
        k = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
        mask = cv2.morphologyEx(raw, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        return mask

    # ======================================================
    # 9. angle 계산
    # ======================================================

    def _normalize_cube_angle(self, angle: float) -> float:
        """
        cube는 90도 대칭입니다.
        그래서 0도와 90도가 사실상 같은 잡기 방향이 될 수 있습니다.

        이 함수는 angle을 -45~45도 범위로 정리합니다.
        """
        while angle > 45:
            angle -= 90

        while angle < -45:
            angle += 90

        return float(angle)

    def _calc_angle(
        self,
        roi_mask: np.ndarray,
        offset: tuple,
        color_roi: np.ndarray = None
    ):
        """
        bbox 내부 ROI에서 실제 물체 외곽에 가까운 contour를 찾고
        cv2.minAreaRect()로 회전각을 계산합니다.

        roi_mask:
            depth 기반 물체 후보 mask
        offset:
            ROI 좌표를 전체 이미지 좌표로 되돌리기 위한 (x, y)
        color_roi:
            color edge를 뽑기 위한 ROI

        반환:
            angle_deg: 회전각
            box_pts: 회전 박스 꼭짓점
            reliable: angle 신뢰도
        """
        if roi_mask.size == 0:
            return 0.0, None, False

        # ROI mask 내부 노이즈 제거
        k = np.ones((3, 3), np.uint8)
        roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, k)
        roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_CLOSE, k)

        # color ROI가 있으면 color edge와 depth mask를 결합합니다.
        # depth mask만 쓰면 경계가 뭉개질 수 있고,
        # edge만 쓰면 테이블 무늬가 섞일 수 있어서 AND로 결합합니다.
        if color_roi is not None and color_roi.size > 0:
            gray = cv2.cvtColor(color_roi, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)

            # edge를 조금 두껍게 만들어 depth/color 정렬 오차를 보완합니다.
            dk = np.ones((EDGE_DILATE_KSIZE, EDGE_DILATE_KSIZE), np.uint8)
            edges = cv2.dilate(edges, dk)

            # depth mask와 color edge가 겹치는 부분만 사용합니다.
            combined = cv2.bitwise_and(roi_mask, edges)

            # combined가 너무 비어 있으면 오히려 불안정하므로 depth mask만 사용합니다.
            if cv2.countNonZero(combined) > MIN_CONTOUR_AREA // 2:
                roi_mask = combined

        # contour 추출
        contours, _ = cv2.findContours(
            roi_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return 0.0, None, False

        # 가장 큰 contour를 실제 물체로 봅니다.
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)

        if area < MIN_CONTOUR_AREA:
            return 0.0, None, False

        # contour를 convex hull로 정리해서 외곽선을 안정화합니다.
        hull = cv2.convexHull(cnt)

        # hull point가 너무 적으면 angle 신뢰도가 낮다고 봅니다.
        reliable = len(hull) >= MIN_HULL_POINTS

        # 물체를 감싸는 최소 회전 사각형을 구합니다.
        rect = cv2.minAreaRect(hull)
        (_, _), (w, h), angle = rect

        # OpenCV minAreaRect angle은 -90~0도 기준이라
        # 로봇에서 쓰기 편하게 -90~90도 기준으로 정규화합니다.
        if w < h:
            angle += 90

        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180

        # 길쭉한 물체는 angle이 비교적 안정적이고,
        # 정사각형에 가까운 물체는 angle이 튈 수 있습니다.
        long_side = max(w, h)
        short_side = min(w, h)
        aspect = long_side / max(short_side, 1e-5)

        if aspect < MIN_ASPECT_RATIO and aspect > 0:
            reliable = False

        # minAreaRect의 꼭짓점 좌표는 ROI 내부 좌표입니다.
        # 전체 이미지 기준 좌표로 바꾸기 위해 offset을 더합니다.
        ox, oy = offset
        box_local = cv2.boxPoints(rect).astype(int)
        box_pts = box_local + np.array([ox, oy])

        return float(angle), box_pts, bool(reliable)

    # ======================================================
    # 10. angle smoothing
    # ======================================================

    def _smooth_angle(
        self,
        label: str,
        obj_id: int,
        angle: float,
        reliable: bool
    ) -> float:
        """
        angle 값이 프레임마다 흔들리지 않도록 smoothing합니다.

        reliable=False:
            새 angle을 히스토리에 넣지 않고 이전 평균값 유지
        reliable=True지만 갑자기 많이 튐:
            점프 필터로 무시
        """
        key = f"{label}_{obj_id}"

        if key not in self._angle_history:
            self._angle_history[key] = deque(maxlen=ANGLE_SMOOTH_N)

        if key not in self._prev_angles:
            self._prev_angles[key] = angle

        # 신뢰도 낮으면 현재 angle을 버리고 기존 평균을 반환합니다.
        if not reliable:
            return self._get_smoothed_or_current(key, angle)

        # 이전 angle과 현재 angle의 차이를 계산합니다.
        prev = self._prev_angles[key]
        delta = abs(angle - prev)
        delta = min(delta, 180 - delta)

        # angle이 갑자기 너무 크게 변하면 잘못된 검출로 보고 무시합니다.
        if delta > MAX_ANGLE_JUMP_DEG and len(self._angle_history[key]) > 0:
            return self._get_smoothed_or_current(key, angle)

        # 정상 angle이면 히스토리에 추가합니다.
        self._angle_history[key].append(math.radians(angle))
        self._prev_angles[key] = angle

        return self._get_smoothed_or_current(key, angle)

    def _get_smoothed_or_current(self, key: str, current_angle: float) -> float:
        """
        angle 히스토리가 있으면 circular mean을 반환하고,
        없으면 현재 angle을 반환합니다.

        circular mean을 쓰는 이유:
        -89도와 +89도를 일반 평균하면 0도가 되어버리는 문제가 있기 때문입니다.
        """
        angles = list(self._angle_history[key])

        if not angles:
            return float(current_angle)

        sin_m = sum(math.sin(a) for a in angles) / len(angles)
        cos_m = sum(math.cos(a) for a in angles) / len(angles)

        return float(math.degrees(math.atan2(sin_m, cos_m)))

    # ======================================================
    # 11. 픽셀 좌표 → 카메라 3D 좌표
    # ======================================================

    def _pixel_to_cam(self, u: int, v: int, depth_frame):
        """
        픽셀 좌표 (u, v)와 depth z를 이용해
        카메라 좌표계의 3D 점 [x, y, z, 1]을 만듭니다.
        """
        z = self._depth_median(depth_frame, u, v, DEPTH_MEDIAN_KSIZE)

        if z <= 0:
            return None

        # pinhole camera model 공식입니다.
        x = (u - self.ppx) * z / self.fx
        y = (v - self.ppy) * z / self.fy

        return np.array([x, y, z, 1.0], dtype=np.float64)

    def _depth_median(self, depth_frame, u: int, v: int, ksize: int):
        """
        중심 픽셀 하나만 쓰면 depth 노이즈가 커서,
        주변 ksize x ksize 영역의 유효 depth median을 사용합니다.
        """
        half = ksize // 2
        vals = []

        for c in range(u - half, u + half + 1):
            for r in range(v - half, v + half + 1):
                if 0 <= c < STREAM_WIDTH and 0 <= r < STREAM_HEIGHT:
                    d = depth_frame.get_distance(c, r)
                    if d > 0:
                        vals.append(d)

        if not vals:
            return 0.0

        return float(np.median(vals))

    # ======================================================
    # 12. 물체 결과 평균화 / 안정화
    # ======================================================

    def _stabilize_objects(self, objects):
        """
        현재 프레임에서 검출된 objects를 기존 track과 매칭하고,
        같은 물체의 최근 여러 프레임 값을 평균내서 안정화합니다.

        왜 필요한가?
        - YOLO bbox 중심이 프레임마다 조금씩 흔들림
        - depth 값도 픽셀 단위로 흔들림
        - minAreaRect angle도 contour 변화 때문에 흔들림
        그래서 로봇팔에 보낼 때는 최근 N프레임 평균값을 쓰는 편이 안정적입니다.
        """
        stabilized = []
        used_tracks = set()

        for obj in objects:
            label = obj["label"]
            px, py, pz = obj["position_base"]
            pos = np.array([px, py, pz], dtype=np.float64)

            # 현재 obj와 가장 가까운 기존 track을 찾습니다.
            best_key = None
            best_dist = float("inf")

            for key, track in self._object_tracks.items():
                if key in used_tracks:
                    continue

                # label이 다르면 같은 물체로 보지 않습니다.
                if track["label"] != label:
                    continue

                last_pos = np.array(track["last_position"], dtype=np.float64)
                dist = float(np.linalg.norm(pos - last_pos))

                if dist < best_dist:
                    best_dist = dist
                    best_key = key

            # 가까운 track이 없으면 새 물체로 등록합니다.
            if best_key is None or best_dist > OBJECT_MATCH_DIST:
                best_key = f"{label}_{self._next_track_id}"
                self._next_track_id += 1
                self._object_tracks[best_key] = {
                    "label": label,
                    "history": deque(maxlen=OBJECT_AVG_N),
                    "last_position": obj["position_base"],
                }

            used_tracks.add(best_key)
            track = self._object_tracks[best_key]
            track["last_position"] = obj["position_base"]

            # 현재 관측값을 히스토리에 저장합니다.
            track["history"].append({
                "position_base": obj["position_base"],
                "angle_deg": obj["angle_deg"],
                "conf": obj["conf"],
                "area": obj["area"],
                "center_px": obj["center_px"],
                "bbox": obj["bbox"],
                "reliable": obj.get("reliable", True),
            })

            hist = list(track["history"])

            # 관측 횟수가 너무 적으면 평균은 내되, stable 표시는 낮게 봅니다.
            obs_count = len(hist)

            # position 평균
            positions = np.array([h["position_base"] for h in hist], dtype=np.float64)
            mean_pos = positions.mean(axis=0)

            # angle 평균은 일반 평균 대신 circular mean을 사용합니다.
            # 예: -89도와 +89도의 평균이 0도가 되는 문제를 막기 위함입니다.
            angle_rad = [math.radians(h["angle_deg"]) for h in hist]
            sin_m = sum(math.sin(a) for a in angle_rad) / len(angle_rad)
            cos_m = sum(math.cos(a) for a in angle_rad) / len(angle_rad)
            mean_angle = math.degrees(math.atan2(sin_m, cos_m))

            # confidence, area 평균
            mean_conf = float(np.mean([h["conf"] for h in hist]))
            mean_area = float(np.mean([h["area"] for h in hist]))

            # pixel 중심 평균
            mean_cx = int(np.mean([h["center_px"][0] for h in hist]))
            mean_cy = int(np.mean([h["center_px"][1] for h in hist]))

            # bbox는 마지막 값을 사용합니다.
            # 평균 bbox도 가능하지만, 화면 표시용이므로 최신값이 자연스럽습니다.
            latest = hist[-1]

            # reliable은 최근 히스토리 중 reliable=True가 절반 이상이면 True로 봅니다.
            reliable_count = sum(1 for h in hist if h.get("reliable", True))
            avg_reliable = reliable_count >= max(1, len(hist) // 2)

            stable_obj = dict(obj)
            stable_obj["id"] = int(best_key.split("_")[-1])
            stable_obj["position_base"] = (
                float(mean_pos[0]),
                float(mean_pos[1]),
                float(mean_pos[2]),
            )
            stable_obj["angle_deg"] = float(mean_angle)
            stable_obj["conf"] = mean_conf
            stable_obj["area"] = mean_area
            stable_obj["center_px"] = (mean_cx, mean_cy)
            stable_obj["bbox"] = latest["bbox"]
            stable_obj["reliable"] = bool(avg_reliable and obs_count >= OBJECT_MIN_OBS)
            stable_obj["obs_count"] = obs_count

            stabilized.append(stable_obj)

        return stabilized

    # ======================================================
    # 13. 디버그 화면 그리기
    # ======================================================

    def _draw_debug(self, color_img, objects):
        """
        OpenCV 창에 검출 결과를 그립니다.

        표시 내용:
        - YOLO bbox
        - minAreaRect 회전 박스
        - angle 화살표
        - label/confidence
        - base 좌표
        """
        overlay = color_img.copy()

        for obj in objects:
            x, y, w, h = obj["bbox"]
            cx, cy = obj["center_px"]

            label = obj["label"]
            conf = obj["conf"]
            angle = obj["angle_deg"]
            box_pts = obj["box_pts"]
            reliable = obj.get("reliable", True)

            color = CLASS_COLORS.get(label, (200, 200, 200))

            # YOLO bbox입니다.
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color, 1)

            # minAreaRect 회전 박스입니다.
            # angle 신뢰도가 높으면 초록, 낮으면 주황으로 표시합니다.
            if box_pts is not None:
                color_box = (0, 255, 0) if reliable else (0, 165, 255)
                cv2.drawContours(overlay, [box_pts], 0, color_box, 2)

            # angle 방향 화살표입니다.
            arrow_len = max(w, h) // 2
            ex = int(cx + arrow_len * math.cos(math.radians(angle)))
            ey = int(cy + arrow_len * math.sin(math.radians(angle)))

            cv2.arrowedLine(
                overlay,
                (cx, cy),
                (ex, ey),
                (0, 0, 255),
                2,
                tipLength=0.3
            )

            # label과 confidence 표시
            cv2.putText(
                overlay,
                f"{label} {conf:.2f}",
                (x, max(y - 22, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA
            )

            # angle 표시
            angle_text = f"angle: {angle:.1f}deg"
            if not reliable:
                angle_text += " (low)"

            cv2.putText(
                overlay,
                angle_text,
                (x, max(y - 5, 36)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
                cv2.LINE_AA
            )

            # 로봇 base 좌표 표시
            px, py, pz = obj["position_base"]

            cv2.putText(
                overlay,
                f"base=({px:.3f},{py:.3f},{pz:.3f})",
                (x, y + h + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                color,
                1,
                cv2.LINE_AA
            )

        cv2.putText(
            overlay,
            f"objects={len(objects)}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2
        )

        cv2.imshow(DEBUG_WINDOW, overlay)
        cv2.waitKey(1)

    # ======================================================
    # 14. dict → ObjectInfo 메시지 변환
    # ======================================================

    def _make_msg(self, obj) -> ObjectInfo:
        """
        내부 처리용 dict를 ROS2 메시지 ObjectInfo로 바꿉니다.

        이 메시지가 서비스 응답으로 로봇팔 노드에 전달됩니다.
        """
        cx, cy = obj["center_px"]
        x, y, w, h = obj["bbox"]
        px, py, pz = obj["position_base"]

        log_info(
            self,
            f"[{obj['label']}] "
            f"conf={obj['conf']:.2f}, "
            f"angle={obj['angle_deg']:.1f}deg, "
            f"reliable={obj.get('reliable', True)}, "
            f"obs={obj.get('obs_count', 1)}, "
            f"base=({px:.3f},{py:.3f},{pz:.3f})"
        )

        info = ObjectInfo()

        # 물체 번호와 이름
        info.id = int(obj["id"])
        info.label = str(obj["label"])

        # 로봇 base 좌표계 기준 위치
        info.position = Vector3(
            x=float(px),
            y=float(py),
            z=float(pz)
        )

        # 로봇 gripper rz에 사용할 수 있는 회전각
        info.angle_deg = float(obj["angle_deg"])

        # angle을 믿을 수 있는지 여부
        info.angle_reliable = bool(obj.get("reliable", True))

        # 이미지 상 중심 픽셀
        info.pixel_x = int(cx)
        info.pixel_y = int(cy)

        # YOLO bbox 정보
        info.bbox_x = int(x)
        info.bbox_y = int(y)
        info.bbox_w = int(w)
        info.bbox_h = int(h)

        # bbox 면적과 YOLO confidence
        info.area = float(obj["area"])
        info.confidence = float(obj["conf"])

        return info

    # ======================================================
    # 15. 종료 처리
    # ======================================================

    def destroy_node(self):
        """
        노드 종료 시 카메라와 OpenCV 창을 정리합니다.
        """
        try:
            self.pipeline.stop()
        except Exception:
            pass

        cv2.destroyAllWindows()
        super().destroy_node()


# ==========================================================
# 16. main 함수
# ==========================================================

def main(args=None):
    rclpy.init(args=args)

    node = ObjectContourServiceServer()

    try:
        # 서비스 요청을 계속 기다립니다.
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
