import rclpy
from rclpy.node import Node
import numpy as np
import pyrealsense2 as rs
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        
        # 1. ROS 2 퍼블리셔 및 CvBridge 설정
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/color/camera_info', 10)

        # 2. RealSense 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        
        # 카메라 내부 파라미터 추출
        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_stream.get_intrinsics()
        
        # CameraInfo 메시지 생성 (미리 만들어두고 계속 사용)
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_color_optical_frame"
        self.camera_info_msg.width = self.intrinsics.width
        self.camera_info_msg.height = self.intrinsics.height
        self.camera_info_msg.k = [self.intrinsics.fx, 0.0, self.intrinsics.ppx, 
                                  0.0, self.intrinsics.fy, self.intrinsics.ppy, 
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.d = self.intrinsics.coeffs

        # 3. 타이머 기반 프레임 캡처 및 퍼블리시 (30FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frames)
        self.get_logger().info(">>> RealSense Camera Node is Publishing Frames.")

    def publish_frames(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return

            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())

            # 타임스탬프 동기화
            now = self.get_clock().now().to_msg()
            self.camera_info_msg.header.stamp = now

            # Image 메시지 변환
            color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="16UC1") # 16비트 Depth
            
            color_msg.header.stamp = now
            color_msg.header.frame_id = "camera_color_optical_frame"
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "camera_depth_optical_frame"

            # 퍼블리시
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            self.info_pub.publish(self.camera_info_msg)

        except Exception as e:
            self.get_logger().error(f"Frame capture error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()