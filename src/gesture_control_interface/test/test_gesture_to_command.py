import unittest
import rclpy
from gesture_control_interface.gesture_to_command import GestureToCommandNode


class TestGestureToCommand(unittest.TestCase):
    """Test cases for gesture to command mapping"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = GestureToCommandNode()
    
    def tearDown(self):
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """Test node initializes correctly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'gesture_to_command')
    
    def test_default_mappings_loaded(self):
        """Test default mappings are loaded"""
        self.assertIsNotNone(self.node.gesture_mappings)
        self.assertIn('basic', self.node.gesture_mappings)
    
    def test_gesture_mapping(self):
        """Test gesture maps to correct command"""
        command = self.node.map_gesture_to_command('wave', 0.85)
        self.assertIsNotNone(command)
        self.assertEqual(command.action, 'home')
    
    def test_invalid_gesture(self):
        """Test invalid gesture returns None"""
        command = self.node.map_gesture_to_command('invalid_gesture', 0.85)
        self.assertIsNone(command)


class TestModeChanging(unittest.TestCase):
    """Test mode changing functionality"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = GestureToCommandNode()
    
    def tearDown(self):
        self.node.destroy_node()
    
    def test_mode_change(self):
        """Test mode can be changed"""
        initial_mode = self.node.control_mode
        self.node.control_mode = 'advanced'
        self.assertEqual(self.node.control_mode, 'advanced')
        self.assertNotEqual(self.node.control_mode, initial_mode)


if __name__ == '__main__':
    unittest.main()
