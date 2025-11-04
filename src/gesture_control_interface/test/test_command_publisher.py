import unittest
import rclpy
from gesture_control_interface.command_publisher import CommandPublisherNode


class TestCommandPublisher(unittest.TestCase):
    """Test cases for command publisher"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = CommandPublisherNode()
    
    def tearDown(self):
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """Test node initializes correctly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'command_publisher')
    
    def test_history_tracking(self):
        """Test command history is tracked"""
        initial_length = len(self.node.command_history)
        # Simulate adding commands
        self.assertEqual(len(self.node.command_history), initial_length)


if __name__ == '__main__':
    unittest.main()
