import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraHandlerNode(Node):
    """
    Node that captures camera frames and publishes them as ROS2 Image messages.
    """
    
    def __init__(self):
        super().__init__('camera_handler')
        
        # Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('video_file', '')
        self.declare_parameter('output_topic', '/camera/image_raw')
        
        camera_index = self.get_parameter('camera_index').value
        width = self.get_parameter('camera_width').value
        height = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value
        video_file = self.get_parameter('video_file').value
        output_topic = self.get_parameter('output_topic').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Open camera or video file
        if video_file:
            self.cap = cv2.VideoCapture(video_file)
            self.get_logger().info(f'Opening video file: {video_file}')
        else:
            self.cap = cv2.VideoCapture(camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.get_logger().info(f'Opening camera {camera_index}')
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera/video!')
            return
        
        # Publisher
        self.image_pub = self.create_publisher(Image, output_topic, 10)
        
        # Timer for frame capture
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Camera handler initialized, publishing to {output_topic}')
    
    def timer_callback(self):
        """Capture and publish frame"""
        ret, frame = self.cap.read()
        
        if ret:
            try:
                # Convert to ROS Image message
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                
                # Publish
                self.image_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {str(e)}')
        else:
            self.get_logger().warn('Failed to capture frame')
    
    def destroy_node(self):
        """Cleanup"""
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraHandlerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()