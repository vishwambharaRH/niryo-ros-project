import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import deque


class GestureClassifierNode(Node):
    """
    Classifier that processes gesture detections and applies temporal filtering
    to reduce false positives and improve reliability.
    """
    
    def __init__(self):
        super().__init__('gesture_classifier')
        
        # Parameters
        self.declare_parameter('confidence_window_size', 5)
        self.declare_parameter('min_consecutive_detections', 3)
        self.declare_parameter('output_topic', '/gesture/classified')
        
        self.window_size = self.get_parameter('confidence_window_size').value
        self.min_consecutive = self.get_parameter('min_consecutive_detections').value
        output_topic = self.get_parameter('output_topic').value
        
        # State tracking
        self.gesture_buffer = deque(maxlen=self.window_size)
        self.confidence_buffer = deque(maxlen=self.window_size)
        self.last_published_gesture = "none"
        
        # Publishers and Subscribers
        self.classified_pub = self.create_publisher(String, output_topic, 10)
        self.raw_sub = self.create_subscription(
            String,
            '/gesture/detected',
            self.gesture_callback,
            10
        )
        
        self.get_logger().info('Gesture Classifier Node initialized')
    
    def gesture_callback(self, msg):
        """Process incoming gesture detections"""
        try:
            data = json.loads(msg.data)
            gesture = data['gesture']
            confidence = data['confidence']
            
            # Add to buffers
            self.gesture_buffer.append(gesture)
            self.confidence_buffer.append(confidence)
            
            # Classify based on temporal consistency
            classified_gesture = self.classify_gesture()
            
            # Publish if different from last
            if classified_gesture != self.last_published_gesture:
                self.publish_classified_gesture(classified_gesture, data)
                self.last_published_gesture = classified_gesture
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Error parsing gesture data: {str(e)}')
    
    def classify_gesture(self):
        """
        Classify gesture based on temporal consistency.
        Requires minimum consecutive detections of the same gesture.
        """
        if len(self.gesture_buffer) < self.min_consecutive:
            return "none"
        
        # Get recent gestures
        recent_gestures = list(self.gesture_buffer)[-self.min_consecutive:]
        
        # Check if all are the same and not "none"
        if len(set(recent_gestures)) == 1 and recent_gestures[0] != "none":
            # Calculate average confidence
            recent_confidences = list(self.confidence_buffer)[-self.min_consecutive:]
            avg_confidence = sum(recent_confidences) / len(recent_confidences)
            
            if avg_confidence > 0.7:
                return recent_gestures[0]
        
        return "none"
    
    def publish_classified_gesture(self, gesture, original_data):
        """Publish classified gesture"""
        output_data = {
            'gesture': gesture,
            'confidence': original_data.get('confidence', 0.0),
            'timestamp': original_data.get('timestamp', 0),
            'classification': 'filtered'
        }
        
        if 'hand_position' in original_data:
            output_data['hand_position'] = original_data['hand_position']
        
        msg = String()
        msg.data = json.dumps(output_data)
        self.classified_pub.publish(msg)
        
        self.get_logger().info(f'Classified gesture: {gesture}')


def main(args=None):
    rclpy.init(args=args)
    node = GestureClassifierNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()