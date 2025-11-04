import rclpy
from rclpy.node import Node
from gesture_control_interface.msg import RobotCommand
from std_msgs.msg import String
import json


class CommandPublisherNode(Node):
    """
    Optional utility node that can republish commands in different formats
    or add additional processing/filtering to commands.
    """
    
    def __init__(self):
        super().__init__('command_publisher')
        
        # Declare parameters
        self.declare_parameter('enable_logging', True)
        self.declare_parameter('output_format', 'json')  # json, string
        self.declare_parameter('output_topic', '/robot/command_formatted')
        
        # Get parameters
        self.enable_logging = self.get_parameter('enable_logging').value
        self.output_format = self.get_parameter('output_format').value
        output_topic = self.get_parameter('output_topic').value
        
        # Command history
        self.command_history = []
        self.max_history = 100
        
        # Publishers
        self.formatted_pub = self.create_publisher(
            String,
            output_topic,
            10
        )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            RobotCommand,
            '/robot/command',
            self.command_callback,
            10
        )
        
        self.get_logger().info('Command Publisher Node initialized')
    
    def command_callback(self, msg):
        """Process and republish commands"""
        # Store in history
        self.add_to_history(msg)
        
        # Log if enabled
        if self.enable_logging:
            self.get_logger().info(
                f'Command received: {msg.action} (confidence: {msg.confidence:.2f})'
            )
        
        # Format and republish
        formatted_msg = self.format_command(msg)
        self.formatted_pub.publish(formatted_msg)
    
    def format_command(self, command):
        """Format command based on output format"""
        data = {
            'action': command.action,
            'confidence': command.confidence,
            'mode': command.mode,
            'timestamp': {
                'sec': command.timestamp.sec,
                'nanosec': command.timestamp.nanosec
            },
            'parameters': json.loads(command.parameters) if command.parameters else {}
        }
        
        msg = String()
        
        if self.output_format == 'json':
            msg.data = json.dumps(data, indent=2)
        else:
            msg.data = f"Action: {command.action}, Confidence: {command.confidence:.2f}"
        
        return msg
    
    def add_to_history(self, command):
        """Add command to history"""
        self.command_history.append({
            'action': command.action,
            'confidence': command.confidence,
            'timestamp': self.get_clock().now().to_msg()
        })
        
        # Trim history if too long
        if len(self.command_history) > self.max_history:
            self.command_history.pop(0)
    
    def get_history(self):
        """Get command history"""
        return self.command_history


def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
