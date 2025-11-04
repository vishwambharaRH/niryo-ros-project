import unittest
import rclpy
from gesture_recognition.gesture_classifier import GestureClassifierNode


class TestGestureClassifier(unittest.TestCase):
    """Test cases for gesture classifier"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2"""
        rclpy.shutdown()
    
    def setUp(self):
        """Setup test node"""
        self.node = GestureClassifierNode()
    
    def tearDown(self):
        """Cleanup"""
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """Test that node initializes correctly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'gesture_classifier')
    
    def test_temporal_filtering(self):
        """Test temporal filtering logic"""
        # Simulate multiple detections
        test_gestures = ['thumbs_up'] * 5
        
        for gesture in test_gestures:
            self.node.gesture_buffer.append(gesture)
            self.node.confidence_buffer.append(0.85)
        
        classified = self.node.classify_gesture()
        self.assertEqual(classified, 'thumbs_up')
    
    def test_insufficient_detections(self):
        """Test that insufficient detections return none"""
        self.node.gesture_buffer.append('thumbs_up')
        self.node.confidence_buffer.append(0.85)
        
        classified = self.node.classify_gesture()
        self.assertEqual(classified, 'none')
    
    def test_inconsistent_detections(self):
        """Test that inconsistent detections are filtered"""
        test_data = [
            ('thumbs_up', 0.8),
            ('wave', 0.7),
            ('thumbs_down', 0.9)
        ]
        
        for gesture, conf in test_data:
            self.node.gesture_buffer.append(gesture)
            self.node.confidence_buffer.append(conf)
        
        classified = self.node.classify_gesture()
        self.assertEqual(classified, 'none')


if __name__ == '__main__':
    unittest.main()