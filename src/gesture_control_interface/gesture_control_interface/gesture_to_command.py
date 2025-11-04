import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gesture_control_interface.msg import RobotCommand
from gesture_control_interface.srv import SetGestureMode
import json
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class GestureToCommandNode(Node):
    """
    Node that maps classified gestures to robot commands.
    Supports different control modes and customizable gesture mappings.
    """
    
    def __init__(self):
        super().__init__('gesture_to_command')
        
        # Declare parameters
        self.declare_parameter('gesture_mappings_file', '')
        self.declare_parameter('control_mode', 'basic')  # basic, advanced, custom
        self.declare_parameter('command_topic', '/robot/command')
        
        # Get parameters
        mappings_file = self.get_parameter('gesture_mappings_file').value
        self.control_mode = self.get_parameter('control_mode').value
        command_topic = self.get_parameter('command_topic').value
        
        # Load gesture mappings
        self.gesture_mappings = self.load_gesture_mappings(mappings_file)
        
        # State tracking
        self.last_gesture = "none"
        self.gesture_count = {}
        self.active = True
        
        # Publishers
        self.command_pub = self.create_publisher(
            RobotCommand,
            command_topic,
            10
        )
        
        # Subscribers
        self.gesture_sub = self.create_subscription(
            String,
            '/gesture/classified',
            self.gesture_callback,
            10
        )
        
        # Services
        self.mode_service = self.create_service(
            SetGestureMode,
            'set_gesture_mode',
            self.set_mode_callback
        )
        
        self.get_logger().info(f'Gesture to Command Node initialized in {self.control_mode} mode')
    
    def load_gesture_mappings(self, mappings_file):
        """Load gesture mappings from YAML file"""
        if not mappings_file:
            # Use default mappings file
            pkg_dir = get_package_share_directory('gesture_control_interface')
            mappings_file = os.path.join(pkg_dir, 'config', 'gesture_mappings.yaml')
        
        try:
            with open(mappings_file, 'r') as f:
                mappings = yaml.safe_load(f)
                self.get_logger().info(f'Loaded gesture mappings from {mappings_file}')
                return mappings
        except Exception as e:
            self.get_logger().error(f'Failed to load mappings: {str(e)}')
            return self.get_default_mappings()
    
    def get_default_mappings(self):
        """Return default gesture mappings"""
        return {
            'basic': {
                'wave': {
                    'action': 'home',
                    'description': 'Move to home position'
                },
                'thumbs_up': {
                    'action': 'pick',
                    'description': 'Execute pick action'
                },
                'thumbs_down': {
                    'action': 'place',
                    'description': 'Execute place action'
                }
            },
            'advanced': {
                'wave': {
                    'action': 'calibrate',
                    'description': 'Calibrate robot'
                },
                'thumbs_up': {
                    'action': 'execute_sequence',
                    'description': 'Execute predefined sequence'
                },
                'thumbs_down': {
                    'action': 'stop',
                    'description': 'Emergency stop'
                }
            }
        }
    
    def gesture_callback(self, msg):
        """Process incoming classified gestures"""
        if not self.active:
            return
        
        try:
            data = json.loads(msg.data)
            gesture = data['gesture']
            confidence = data.get('confidence', 0.0)
            
            # Ignore "none" gestures
            if gesture == "none":
                return
            
            # Prevent duplicate processing
            if gesture == self.last_gesture:
                return
            
            self.last_gesture = gesture
            
            # Update gesture count
            self.gesture_count[gesture] = self.gesture_count.get(gesture, 0) + 1
            
            # Map gesture to command
            command = self.map_gesture_to_command(gesture, confidence)
            
            if command:
                self.publish_command(command)
                self.get_logger().info(
                    f'Gesture "{gesture}" mapped to command "{command.action}"'
                )
            
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Error parsing gesture: {str(e)}')
    
    def map_gesture_to_command(self, gesture, confidence):
        """Map a gesture to a robot command"""
        mode_mappings = self.gesture_mappings.get(self.control_mode, {})
        
        if gesture not in mode_mappings:
            self.get_logger().warn(
                f'No mapping for gesture "{gesture}" in mode "{self.control_mode}"'
            )
            return None
        
        mapping = mode_mappings[gesture]
        
        # Create RobotCommand message
        command = RobotCommand()
        command.action = mapping['action']
        command.confidence = confidence
        command.timestamp = self.get_clock().now().to_msg()
        command.mode = self.control_mode
        
        # Add optional parameters
        if 'parameters' in mapping:
            command.parameters = json.dumps(mapping['parameters'])
        else:
            command.parameters = '{}'
        
        return command
    
    def publish_command(self, command):
        """Publish robot command"""
        self.command_pub.publish(command)
    
    def set_mode_callback(self, request, response):
        """Service callback to change control mode"""
        new_mode = request.mode
        
        if new_mode in self.gesture_mappings:
            self.control_mode = new_mode
            response.success = True
            response.message = f'Mode changed to {new_mode}'
            self.get_logger().info(f'Control mode changed to: {new_mode}')
        else:
            response.success = False
            response.message = f'Invalid mode: {new_mode}'
            self.get_logger().warn(f'Invalid mode requested: {new_mode}')
        
        return response
    
    def get_statistics(self):
        """Get gesture statistics"""
        return {
            'total_gestures': sum(self.gesture_count.values()),
            'gesture_counts': self.gesture_count,
            'current_mode': self.control_mode
        }


def main(args=None):
    rclpy.init(args=args)
    node = GestureToCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
