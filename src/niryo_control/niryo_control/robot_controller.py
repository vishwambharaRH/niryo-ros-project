#!/usr/bin/env python3
"""
Abstract base class for robot controllers.
Provides interface for both real and simulated Niryo robots.
"""

from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from gesture_control_interface.msg import RobotCommand
from std_msgs.msg import String
import json


class RobotController(Node, ABC):
    """
    Abstract base class for robot control.
    Subclasses must implement robot-specific command execution.
    """
    
    def __init__(self, node_name='robot_controller'):
        super().__init__(node_name)
        
        # Declare common parameters
        self.declare_parameter('command_topic', '/robot/command')
        self.declare_parameter('status_topic', '/robot/status')
        self.declare_parameter('speed_scale', 0.5)
        
        # Get parameters
        command_topic = self.get_parameter('command_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.speed_scale = self.get_parameter('speed_scale').value
        
        # State tracking
        self.current_action = "idle"
        self.is_busy = False
        self.last_error = None
        
        # Subscribe to commands
        self.command_sub = self.create_subscription(
            RobotCommand,
            command_topic,
            self.command_callback,
            10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10
        )
        
        # Status timer (publish status every second)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f'{node_name} initialized')
    
    def command_callback(self, msg):
        """Process incoming robot commands"""
        if self.is_busy:
            self.get_logger().warn(
                f'Robot busy, ignoring command: {msg.action}'
            )
            return
        
        self.get_logger().info(
            f'Received command: {msg.action} (confidence: {msg.confidence:.2f})'
        )
        
        try:
            # Parse parameters
            parameters = json.loads(msg.parameters) if msg.parameters else {}
            
            # Execute command
            self.execute_command(msg.action, parameters)
            
        except Exception as e:
            self.last_error = str(e)
            self.get_logger().error(f'Command execution failed: {str(e)}')
            self.is_busy = False
    
    def execute_command(self, action, parameters):
        """Route command to appropriate handler"""
        self.is_busy = True
        self.current_action = action
        
        try:
            if action == 'home':
                self.move_home(parameters)
            elif action == 'pick':
                self.execute_pick(parameters)
            elif action == 'place':
                self.execute_place(parameters)
            elif action == 'stop':
                self.emergency_stop(parameters)
            elif action == 'calibrate':
                self.calibrate(parameters)
            elif action == 'execute_sequence':
                self.execute_sequence(parameters)
            else:
                self.get_logger().warn(f'Unknown action: {action}')
        
        except Exception as e:
            self.last_error = str(e)
            self.get_logger().error(f'Action {action} failed: {str(e)}')
        
        finally:
            self.is_busy = False
            self.current_action = "idle"
    
    def publish_status(self):
        """Publish robot status"""
        status = {
            'current_action': self.current_action,
            'is_busy': self.is_busy,
            'last_error': self.last_error,
            'speed_scale': self.speed_scale
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    # Abstract methods to be implemented by subclasses
    
    @abstractmethod
    def move_home(self, parameters):
        """Move robot to home position"""
        pass
    
    @abstractmethod
    def execute_pick(self, parameters):
        """Execute pick operation"""
        pass
    
    @abstractmethod
    def execute_place(self, parameters):
        """Execute place operation"""
        pass
    
    @abstractmethod
    def emergency_stop(self, parameters):
        """Emergency stop all movements"""
        pass
    
    @abstractmethod
    def calibrate(self, parameters):
        """Calibrate robot"""
        pass
    
    @abstractmethod
    def execute_sequence(self, parameters):
        """Execute predefined sequence"""
        pass
    
    @abstractmethod
    def get_current_pose(self):
        """Get current robot pose"""
        pass
    
    @abstractmethod
    def is_connected(self):
        """Check if robot is connected"""
        pass