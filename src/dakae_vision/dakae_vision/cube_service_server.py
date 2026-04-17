import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from dakae_interfaces.srv import DetectCubes
from dakae_interfaces.msg import CubeInfo
from geometry_msgs.msg import Vector3
from google import genai
from google.genai import types

class CubeServiceServer(Node):
    def __init__(self):
        super().__init__('cube_service_server')
        
        # 1. 환경 설정 및 API 클라이언트

        # self.api_key = ""
        self.api_key = ""
        
        self.client = genai.Client(api_key=self.api_key)
        
        self.T_base_cam = np.array([
            [-0.99952167, -0.02565113, -0.00438063,  0.72058381],
            [-0.02557996,  0.99941226, -0.02027756, -0.0018536 ],
            [ 0.00489439, -0.02019912, -0.99960142,  0.69734538],
            [ 0.        ,  0.        ,  0.        ,  1.        ]
        ])

        # 카메라 데이터 수신용
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.fx = self.fy = self.ppx = self.ppy = 0.0

        self.create_subscription(Image, 'camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, 'camera/depth/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, 'camera/color/camera_info', self.info_cb, 10)

        self.srv = self.create_service(DetectCubes, 'detect_cubes', self.detect_cubes_callback)
        self.get_logger().info(">>> Cube Detection Service Server is Ready.")

    def color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def info_cb(self, msg):
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.ppx = msg.k[2]
        self.fy = msg.k[4]
        self.ppy = msg.k[5]

    def get_3d_point(self, u, v, depth_img):
        # 16비트 Depth 이미지는 mm 단위이므로 m로 변환
        z = depth_img[v, u] / 1000.0  
        if z <= 0 or self.fx == 0.0: 
            return None
        
        x = (u - self.ppx) * z / self.fx
        y = (v - self.ppy) * z / self.fy
        return np.array([x, y, z, 1.0])

    def detect_cubes_callback(self, request, response):
        self.get_logger().info(f"Received request for colors: {request.target_colors}")
        
        if self.latest_color is None or self.latest_depth is None:
            response.success = False
            response.message = "Camera frames not received yet."
            return response

        try:
            # 서비스 호출 시점의 최신 프레임 고정
            img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()
            h, w, _ = img.shape

            _, buffer = cv2.imencode('.jpg', img)
            image_bytes = buffer.tobytes()

            prompt = (
                "Locate all cubes(Return the exact center point of the top surface of the cube). Return in JSON: "
                "[{\"point\": [y, x], \"label\": \"color_cube\"}]. "
                "Example label: 'white_cube', 'red_cube'. Points normalized 0-1000."
            )

            api_res = self.client.models.generate_content(
                model="gemini-robotics-er-1.5-preview",
                contents=[types.Part.from_bytes(data=image_bytes, mime_type='image/jpeg'), prompt],
                config=types.GenerateContentConfig(temperature=0.1)
            )

            raw_json = api_res.text.strip().replace("```json", "").replace("```", "")
            detected_objects = json.loads(raw_json)
            
            cube_list = []
            for obj in detected_objects:
                label = obj['label'].split('_')[0]
                
                if request.target_colors and label not in request.target_colors:
                    continue

                norm_y, norm_x = obj['point']
                u, v = int((norm_x / 1000.0) * w), int((norm_y / 1000.0) * h)
                
                p_cam = self.get_3d_point(u, v, depth_img)
                if p_cam is not None:
                    p_base = self.T_base_cam @ p_cam
                    info = CubeInfo()
                    info.color = label
                    info.position = Vector3(x=p_base[0], y=p_base[1], z=p_base[2])
                    cube_list.append(info)

            response.detected_cubes = cube_list
            response.success = True
            response.message = f"Successfully detected {len(cube_list)} cubes."

        except Exception as e:
            self.get_logger().error(f"Detection failed: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CubeServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
