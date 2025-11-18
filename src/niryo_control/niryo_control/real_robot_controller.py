#!/usr/bin/env python3
"""
Real robot controller for Niryo Ned2.
Interfaces with actual Niryo hardware via PyNiryo or ROS2 control.
"""

import rclpy
from niryo_control.robot_controller import RobotController
import time


class RealRobotController(RobotController):
    """
    Controller for real Niryo Ned2 robot.
    Uses pyniryo2 library for robot communication.
    """
    
    def __init__(self):
        super().__init__('real_robot_controller')
        
        # Declare real robot specific parameters
        self.declare_parameter('robot_ip', '192.168.0.10')
        
        robot_ip = self.get_parameter('robot_ip').value
        
        # Initialize robot connection
        self.robot = None
        self.connected = False
        
        try:
            # Import pyniryo2 (optional dependency)
            from pyniryo2 import NiryoRobot
            
            self.get_logger().info(f'Connecting to Niryo robot at {robot_ip}...')
            self.robot = NiryoRobot(robot_ip)
            self.robot.arm.calibrate_auto()
            self.connected = True
            self.get_logger().info('Successfully connected to Niryo robot')
            
        except ImportError:
            self.get_logger().error(
                'pyniryo2 not installed. Install with: pip install pyniryo2'
            )
            self.get_logger().warn('Running in mock mode (no actual robot control)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {str(e)}')
            self.get_logger().warn('Running in mock mode')
    
    def move_home(self, parameters):
        """Move robot to home position"""
        self.get_logger().info('Moving to home position...')
        
        speed = parameters.get('speed', self.speed_scale * 100)
        
        if self.connected and self.robot:
            try:
                self.robot.arm.move_to_home_pose()
                self.get_logger().info('Reached home position')
            except Exception as e:
                self.get_logger().error(f'Home movement failed: {str(e)}')
        else:
            # Mock behavior
            self.get_logger().info('MOCK: Moving to home position')
            time.sleep(1.5)
    
    def execute_pick(self, parameters):
        """Execute pick operation"""
        self.get_logger().info('Executing pick operation...')
        
        grip_force = parameters.get('grip_force', 500)
        approach_height = parameters.get('approach_height', 0.15)
        
        if self.connected and self.robot:
            try:
                # Get current position
                current_pose = self.robot.arm.get_pose()
                
                # Move above object
                approach_pose = current_pose.copy()
                approach_pose.position.z = approach_height
                self.robot.arm.move_pose(approach_pose)
                
                # Move down to grasp
                grasp_pose = approach_pose.copy()
                grasp_pose.position.z = 0.05
                self.robot.arm.move_pose(grasp_pose)
                
                # Close gripper
                self.robot.tool.close_gripper(speed=500, max_torque=grip_force)
                
                # Lift object
                self.robot.arm.move_pose(approach_pose)
                
                self.get_logger().info('Pick operation completed')
                
            except Exception as e:
                self.get_logger().error(f'Pick operation failed: {str(e)}')
        else:
            # Mock behavior
            self.get_logger().info('MOCK: Executing pick operation')
            time.sleep(2.0)
    
    def execute_place(self, parameters):
        """Execute place operation"""
        self.get_logger().info('Executing place operation...')
        
        release_height = parameters.get('release_height', 0.10)
        
        if self.connected and self.robot:
            try:
                # Get current position
                current_pose = self.robot.arm.get_pose()
                
                # Move above placement location
                approach_pose = current_pose.copy()
                approach_pose.position.z = release_height
                self.robot.arm.move_pose(approach_pose)
                
                # Move down
                place_pose = approach_pose.copy()
                place_pose.position.z = 0.05
                self.robot.arm.move_pose(place_pose)
                
                # Open gripper
                self.robot.tool.open_gripper(speed=500)
                
                # Retreat
                self.robot.arm.move_pose(approach_pose)
                
                self.get_logger().info('Place operation completed')
                
            except Exception as e:
                self.get_logger().error(f'Place operation failed: {str(e)}')
        else:
            # Mock behavior
            self.get_logger().info('MOCK: Executing place operation')
            time.sleep(2.0)
    
    def emergency_stop(self, parameters):
        """Emergency stop all movements"""
        self.get_logger().warn('EMERGENCY STOP')
        
        if self.connected and self.robot:
            try:
                self.robot.arm.stop_move()
                self.get_logger().info('Emergency stop executed')
            except Exception as e:
                self.get_logger().error(f'Emergency stop failed: {str(e)}')
        else:
            self.get_logger().warn('MOCK: Emergency stop')
    
    def calibrate(self, parameters):
        """Calibrate robot"""
        self.get_logger().info('Calibrating robot...')
        
        auto_calibrate = parameters.get('auto_calibrate', True)
        
        if self.connected and self.robot:
            try:
                if auto_calibrate:
                    self.robot.arm.calibrate_auto()
                else:
                    self.robot.arm.calibrate_manual()
                
                self.get_logger().info('Calibration completed')
                
            except Exception as e:
                self.get_logger().error(f'Calibration failed: {str(e)}')
        else:
            self.get_logger().info('MOCK: Calibrating robot')
            time.sleep(3.0)
    
    def execute_sequence(self, parameters):
        """Execute predefined sequence"""
        sequence_id = parameters.get('sequence_id', 1)
        
        self.get_logger().info(f'Executing sequence {sequence_id}...')
        
        if self.connected and self.robot:
            try:
                # Example sequence: wave motion
                if sequence_id == 1:
                    self.robot.arm.move_to_home_pose()
                    
                    # Get home pose
                    home = self.robot.arm.get_pose()
                    
                    # Create wave motion
                    for i in range(3):
                        left_pose = home.copy()
                        left_pose.position.y += 0.05
                        self.robot.arm.move_pose(left_pose)
                        
                        right_pose = home.copy()
                        right_pose.position.y -= 0.05
                        self.robot.arm.move_pose(right_pose)
                    
                    # Return to home
                    self.robot.arm.move_to_home_pose()
                
                self.get_logger().info('Sequence completed')
                
            except Exception as e:
                self.get_logger().error(f'Sequence execution failed: {str(e)}')
        else:
            self.get_logger().info(f'MOCK: Executing sequence {sequence_id}')
            time.sleep(2.5)
    
    def get_current_pose(self):
        """Get current robot pose"""
        if self.connected and self.robot:
            try:
                return self.robot.arm.get_pose()
            except Exception as e:
                self.get_logger().error(f'Failed to get pose: {str(e)}')
                return None
        return None
    
    def is_connected(self):
        """Check if robot is connected"""
        return self.connected
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.connected and self.robot:
            try:
                self.robot.close_connection()
                self.get_logger().info('Robot connection closed')
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = RealRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()