import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
from collections import deque
import json


class GestureDetectorNode(Node):
    """
    ROS2 Node for detecting hand gestures using MediaPipe and OpenCV.
    Detects: Wave, Thumbs Up, Thumbs Down
    """
    
    def __init__(self):
        super().__init__('gesture_detector')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detection_confidence', 0.7)
        self.declare_parameter('tracking_confidence', 0.5)
        self.declare_parameter('gesture_confidence_threshold', 0.75)
        self.declare_parameter('wave_detection_frames', 10)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('flip_camera', False)
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.tracking_confidence = self.get_parameter('tracking_confidence').value
        self.gesture_threshold = self.get_parameter('gesture_confidence_threshold').value
        self.wave_frames = self.get_parameter('wave_detection_frames').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.flip_camera = self.get_parameter('flip_camera').value
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=self.detection_confidence,
            min_tracking_confidence=self.tracking_confidence
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Wave detection buffers
        self.hand_positions = deque(maxlen=self.wave_frames)
        self.wrist_positions = deque(maxlen=self.wave_frames)
        
        # Gesture state
        self.current_gesture = "none"
        self.gesture_confidence = 0.0
        self.last_gesture_time = self.get_clock().now()
        self.gesture_cooldown = 1.0  # seconds
        
        # Publishers
        self.gesture_pub = self.create_publisher(
            String, 
            '/gesture/detected', 
            10
        )
        
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/gesture/debug_image',
                10
            )
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info('Gesture Detector Node initialized')
        self.get_logger().info(f'Listening to camera topic: {camera_topic}')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.flip_camera:
                cv_image = cv2.flip(cv_image, 1)
            
            # Convert to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process image with MediaPipe
            results = self.hands.process(rgb_image)
            
            # Detect gesture
            gesture, confidence, hand_center = self.detect_gesture(results, cv_image.shape)
            
            # Check cooldown
            current_time = self.get_clock().now()
            time_since_last = (current_time - self.last_gesture_time).nanoseconds / 1e9
            
            if gesture != "none" and time_since_last > self.gesture_cooldown:
                if confidence > self.gesture_threshold:
                    self.current_gesture = gesture
                    self.gesture_confidence = confidence
                    self.last_gesture_time = current_time
                    
                    # Publish gesture
                    self.publish_gesture(gesture, confidence, hand_center)
                    self.get_logger().info(f'Detected: {gesture} (confidence: {confidence:.2f})')
            
            # Draw debug visualization
            if self.publish_debug and results.multi_hand_landmarks:
                self.draw_debug_image(cv_image, results, gesture, confidence)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_gesture(self, results, image_shape):
        """
        Detect gesture from hand landmarks
        Returns: (gesture_name, confidence, hand_center_point)
        """
        if not results.multi_hand_landmarks:
            self.hand_positions.clear()
            self.wrist_positions.clear()
            return "none", 0.0, None
        
        hand_landmarks = results.multi_hand_landmarks[0]
        landmarks = hand_landmarks.landmark
        
        # Get key landmarks
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = landmarks[self.mp_hands.HandLandmark.PINKY_TIP]
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        
        # Calculate hand center
        hand_center = Point()
        hand_center.x = wrist.x
        hand_center.y = wrist.y
        hand_center.z = wrist.z
        
        # Store positions for wave detection
        self.wrist_positions.append((wrist.x, wrist.y))
        hand_center_pos = (
            (wrist.x + middle_tip.x) / 2,
            (wrist.y + middle_tip.y) / 2
        )
        self.hand_positions.append(hand_center_pos)
        
        # Check for thumbs up
        thumbs_up, tu_conf = self.is_thumbs_up(landmarks, image_shape)
        if thumbs_up:
            return "thumbs_up", tu_conf, hand_center
        
        # Check for thumbs down
        thumbs_down, td_conf = self.is_thumbs_down(landmarks, image_shape)
        if thumbs_down:
            return "thumbs_down", td_conf, hand_center
        
        # Check for wave
        wave, wave_conf = self.is_waving()
        if wave:
            return "wave", wave_conf, hand_center
        
        return "none", 0.0, hand_center
    
    def is_thumbs_up(self, landmarks, image_shape):
        """Detect thumbs up gesture"""
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = landmarks[self.mp_hands.HandLandmark.THUMB_MCP]
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = landmarks[self.mp_hands.HandLandmark.PINKY_TIP]
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        
        # Thumb extended upward
        thumb_extended = thumb_tip.y < thumb_ip.y < thumb_mcp.y
        
        # Other fingers curled (tips below knuckles)
        fingers_curled = (
            index_tip.y > index_mcp.y and
            middle_tip.y > index_mcp.y and
            ring_tip.y > index_mcp.y and
            pinky_tip.y > index_mcp.y
        )
        
        # Hand orientation roughly upright
        hand_upright = abs(wrist.y - index_mcp.y) > 0.1
        
        if thumb_extended and fingers_curled and hand_upright:
            # Calculate confidence based on how well criteria are met
            thumb_score = (thumb_mcp.y - thumb_tip.y) / (wrist.y - thumb_tip.y + 0.001)
            curl_score = sum([
                1 if tip.y > index_mcp.y else 0 
                for tip in [index_tip, middle_tip, ring_tip, pinky_tip]
            ]) / 4.0
            confidence = (thumb_score + curl_score) / 2.0
            return True, min(confidence, 1.0)
        
        return False, 0.0
    
    def is_thumbs_down(self, landmarks, image_shape):
        """Detect thumbs down gesture"""
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = landmarks[self.mp_hands.HandLandmark.THUMB_MCP]
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = landmarks[self.mp_hands.HandLandmark.PINKY_TIP]
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        
        # Thumb extended downward
        thumb_extended = thumb_tip.y > thumb_ip.y > thumb_mcp.y
        
        # Other fingers curled (tips above knuckles in inverted hand)
        fingers_curled = (
            index_tip.y < index_mcp.y and
            middle_tip.y < index_mcp.y and
            ring_tip.y < index_mcp.y and
            pinky_tip.y < index_mcp.y
        )
        
        # Hand orientation roughly inverted
        hand_inverted = wrist.y < index_mcp.y
        
        if thumb_extended and fingers_curled and hand_inverted:
            # Calculate confidence
            thumb_score = (thumb_tip.y - thumb_mcp.y) / (thumb_tip.y - wrist.y + 0.001)
            curl_score = sum([
                1 if tip.y < index_mcp.y else 0 
                for tip in [index_tip, middle_tip, ring_tip, pinky_tip]
            ]) / 4.0
            confidence = (thumb_score + curl_score) / 2.0
            return True, min(confidence, 1.0)
        
        return False, 0.0
    
    def is_waving(self):
        """Detect waving gesture by tracking hand movement"""
        if len(self.hand_positions) < self.wave_frames:
            return False, 0.0
        
        positions = list(self.hand_positions)
        
        # Calculate horizontal movement
        x_coords = [pos[0] for pos in positions]
        
        # Detect oscillation (multiple direction changes)
        direction_changes = 0
        for i in range(1, len(x_coords) - 1):
            if (x_coords[i] - x_coords[i-1]) * (x_coords[i+1] - x_coords[i]) < 0:
                direction_changes += 1
        
        # Calculate movement amplitude
        x_range = max(x_coords) - min(x_coords)
        
        # Wave detected if multiple direction changes and sufficient movement
        if direction_changes >= 3 and x_range > 0.15:
            # Confidence based on regularity and amplitude
            confidence = min((direction_changes / 5.0 + x_range) / 2.0, 1.0)
            return True, confidence
        
        return False, 0.0
    
    def publish_gesture(self, gesture, confidence, hand_center):
        """Publish detected gesture"""
        msg_data = {
            'gesture': gesture,
            'confidence': float(confidence),
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        if hand_center:
            msg_data['hand_position'] = {
                'x': float(hand_center.x),
                'y': float(hand_center.y),
                'z': float(hand_center.z)
            }
        
        msg = String()
        msg.data = json.dumps(msg_data)
        self.gesture_pub.publish(msg)
    
    def draw_debug_image(self, image, results, gesture, confidence):
        """Draw debug visualization on image"""
        # Draw hand landmarks
        for hand_landmarks in results.multi_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS
            )
        
        # Draw gesture text
        if gesture != "none":
            text = f'{gesture.upper()} ({confidence:.2f})'
            cv2.putText(
                image, text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1,
                (0, 255, 0), 2
            )
        
        # Draw instructions
        cv2.putText(
            image, 'Show: Wave, Thumbs Up, or Thumbs Down',
            (10, image.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 1
        )
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.debug_image_pub.publish(debug_msg)
    
    def destroy_node(self):
        """Cleanup"""
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
