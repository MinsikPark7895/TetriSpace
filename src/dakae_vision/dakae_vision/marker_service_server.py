import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from collections import deque
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from dakae_interfaces.srv import DetectMarkers
from dakae_interfaces.msg import MarkerInfo
from geometry_msgs.msg import Point

class MarkerServiceServer(Node):
    def __init__(self):
        super().__init__('marker_service_server')
        
        self.T_base_cam = np.array([
            [-0.99952167, -0.02565113, -0.00438063,  0.72058381],
            [-0.02557996,  0.99941226, -0.02027756, -0.0018536 ],
            [ 0.00489439, -0.02019912, -0.99960142,  0.69734538],
            [ 0.        ,  0.        ,  0.        ,  1.        ]
        ])

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.SIZE_CUBE = 0.04    
        self.SIZE_TARGET = 0.10  

        self.bridge = CvBridge()
        self.cam_matrix = None
        self.dist_coeffs = np.zeros((5, 1))
        
        # 최신 20프레임을 유지하는 버퍼
        self.frame_buffer = deque(maxlen=20)

        self.create_subscription(Image, 'camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(CameraInfo, 'camera/color/camera_info', self.info_cb, 10)

        self.srv = self.create_service(DetectMarkers, 'detect_markers', self.callback)
        self.get_logger().info(">>> Optimized ArUco Service Server is Ready.")

    def color_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.frame_buffer.append(img)

    def info_cb(self, msg):
        if self.cam_matrix is None:
            self.cam_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)

    def get_normalized_rz(self, rvec, is_cube=True):
        rmat, _ = cv2.Rodrigues(rvec)
        raw_rz = math.degrees(math.atan2(rmat[1, 0], rmat[0, 0]))

        if is_cube:
            return ((raw_rz + 45) % 90) - 45
        else:
            return raw_rz

    def callback(self, request, response):
        if len(self.frame_buffer) < 20 or self.cam_matrix is None:
            response.success = False
            response.message = "Camera initializing or insufficient frames."
            return response

        self.get_logger().info("Request received: Processing 20 cached frames...")
        
        accumulated_data = {} 
        # 버퍼에 있는 20프레임을 복사하여 처리
        frames_to_process = list(self.frame_buffer)

        for img in frames_to_process:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                for i, m_id in enumerate(ids.flatten()):
                    m_id = int(m_id)
                    
                    if 0 <= m_id <= 9:
                        current_size = self.SIZE_CUBE
                        is_cube = True
                    elif 10 <= m_id <= 19:
                        current_size = self.SIZE_TARGET
                        is_cube = False
                    else:
                        continue

                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], current_size, self.cam_matrix, self.dist_coeffs
                    )
                    
                    rz = self.get_normalized_rz(rvecs[0][0], is_cube=is_cube)
                    
                    p_cam = np.array([tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2], 1.0])
                    p_base = self.T_base_cam @ p_cam

                    if m_id not in accumulated_data:
                        accumulated_data[m_id] = []
                    accumulated_data[m_id].append([p_base[0], p_base[1], p_base[2], rz])

        marker_list = []
        for m_id, data_list in accumulated_data.items():
            if len(data_list) < 7: continue 

            avg_vals = np.mean(data_list, axis=0)
            
            info = MarkerInfo()
            if 0 <= m_id <= 9:
                info.type = "cube"
                info.id = m_id
            else:
                info.type = "target"
                info.id = m_id - 10

            info.position = Point(x=avg_vals[0], y=avg_vals[1], z=avg_vals[2])
            info.rz = avg_vals[3]
            marker_list.append(info)

        response.markers = marker_list
        response.success = True
        response.message = f"Detected {len(marker_list)} stable markers."
        return response

def main():
    rclpy.init()
    node = MarkerServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()