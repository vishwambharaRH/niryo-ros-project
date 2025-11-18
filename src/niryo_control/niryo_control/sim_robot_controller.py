#!/usr/bin/env python3
"""
Simulation robot controller for Niryo Ned2.
Interfaces with Gazebo simulation via ROS2 control interfaces.
"""

import rclpy
from niryo_control.robot_controller import RobotController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import time
import math


class SimRobotController(RobotController):
    """
    Controller for simulated Niryo Ned2 robot in Gazebo.
    Uses ROS2 control and joint trajectory controller.
    """
    
    def __init__(self):
        super().__init__('sim_robot_controller')
        
        # Joint names (typical Niryo Ned2 configuration)
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        
        # Current joint states
        self.current_joint_states = None
        self.connected = False
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint trajectory publisher (for basic control)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Action client for trajectory following (more advanced)
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Predefined poses (in radians)
        self.poses = {
            'home': [0.0, 0.3, -1.3, 0.0, 0.0, 0.0],
            'observation': [0.0, -0.5, -0.5, 0.0, -0.5, 0.0],
            'pick_ready': [0.0, 0.0, -1.57, 0.0, 0.0, 0.0],
        }
        
        self.get_logger().info('Simulation robot controller initialized')
        self.get_logger().info('Waiting for joint states...')
        
        # Wait for first joint state message
        self.create_timer(0.1, self.check_connection)
    
    def check_connection(self):
        """Check if we're receiving joint states"""
        if self.current_joint_states is not None and not self.connected:
            self.connected = True
            self.get_logger().info('Connected to simulated robot')
    
    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joint_states = msg
    
    def move_joints(self, joint_positions, duration=2.0):
        """Move joints to target positions"""
        if not self.connected:
            self.get_logger().warn('Not connected to simulation')
            return False
        
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points.append(point)
        
        # Publish trajectory
        self.trajectory_pub.publish(trajectory)
        
        self.get_logger().info(f'Moving joints to: {joint_positions}')
        
        # Wait for movement to complete
        time.sleep(duration + 0.5)
        
        return True
    
    def move_home(self, parameters):
        """Move robot to home position"""
        self.get_logger().info('Moving to home position...')
        
        speed = parameters.get('speed', self.speed_scale * 100)
        duration = 3.0 / (speed / 50.0)
        
        success = self.move_joints(self.poses['home'], duration)
        
        if success:
            self.get_logger().info('Reached home position')
        else:
            self.get_logger().error('Failed to reach home position')
    
    def execute_pick(self, parameters):
        """Execute pick operation"""
        self.get_logger().info('Executing pick operation...')
        
        # Move to pick ready position
        self.move_joints(self.poses['pick_ready'], 2.0)
        
        # Simulate lowering
        pick_pose = self.poses['pick_ready'].copy()
        pick_pose[1] += 0.3  # Lower arm
        self.move_joints(pick_pose, 1.5)
        
        # Simulate gripper close (just wait)
        self.get_logger().info('Closing gripper...')
        time.sleep(1.0)
        
        # Lift
        self.move_joints(self.poses['pick_ready'], 1.5)
        
        self.get_logger().info('Pick operation completed')
    
    def execute_place(self, parameters):
        """Execute place operation"""
        self.get_logger().info('Executing place operation...')
        
        release_height = parameters.get('release_height', 0.10)
        
        # Move to observation position
        self.move_joints(self.poses['observation'], 2.0)
        
        # Simulate lowering to place
        place_pose = self.poses['observation'].copy()
        place_pose[1] += 0.2
        self.move_joints(place_pose, 1.5)
        
        # Simulate gripper open (just wait)
        self.get_logger().info('Opening gripper...')
        time.sleep(1.0)
        
        # Retreat
        self.move_joints(self.poses['observation'], 1.5)
        
        self.get_logger().info('Place operation completed')
    
    def emergency_stop(self, parameters):
        """Emergency stop all movements"""
        self.get_logger().warn('EMERGENCY STOP')
        
        if not self.connected or self.current_joint_states is None:
            self.get_logger().warn('Cannot execute emergency stop - not connected')
            return
        
        # Send current position to stop motion
        current_positions = list(self.current_joint_states.position[:6])
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = current_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds
        
        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)
        
        self.get_logger().info('Emergency stop executed')
    
    def calibrate(self, parameters):
        """Calibrate robot (simulation doesn't need calibration)"""
        self.get_logger().info('Calibrating simulated robot...')
        
        # In simulation, just move to home as "calibration"
        self.move_joints(self.poses['home'], 3.0)
        
        self.get_logger().info('Calibration completed (simulation)')
    
    def execute_sequence(self, parameters):
        """Execute predefined sequence"""
        sequence_id = parameters.get('sequence_id', 1)
        
        self.get_logger().info(f'Executing sequence {sequence_id}...')
        
        if sequence_id == 1:
            # Wave sequence
            self.move_joints(self.poses['home'], 2.0)
            
            # Create waving motion
            for i in range(3):
                wave_left = self.poses['home'].copy()
                wave_left[0] = 0.5  # Joint 1 left
                self.move_joints(wave_left, 0.8)
                
                wave_right = self.poses['home'].copy()
                wave_right[0] = -0.5  # Joint 1 right
                self.move_joints(wave_right, 0.8)
            
            # Return to home
            self.move_joints(self.poses['home'], 2.0)
        
        elif sequence_id == 2:
            # Pick and place sequence
            self.execute_pick({})
            time.sleep(0.5)
            self.execute_place({})
        
        else:
            self.get_logger().warn(f'Unknown sequence ID: {sequence_id}')
        
        self.get_logger().info('Sequence completed')
    
    def get_current_pose(self):
        """Get current robot joint positions"""
        if self.current_joint_states:
            return list(self.current_joint_states.position[:6])
        return None
    
    def is_connected(self):
        """Check if robot is connected"""
        return self.connected


def main(args=None):
    rclpy.init(args=args)
    controller = SimRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()