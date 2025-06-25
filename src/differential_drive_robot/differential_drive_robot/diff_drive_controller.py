#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # Robot parameters
        self.wheel_radius = 0.08  # meters
        self.wheel_separation = 0.45  # meters (distance between wheels)
        
        # State variables
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer for publishing joint states and odometry
        self.timer = self.create_timer(0.02, self.update_robot_state)  # 50Hz
        
        self.get_logger().info('Differential Drive Controller started')
    
    def cmd_vel_callback(self, msg):
        """Convert twist commands to wheel velocities"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Calculate wheel velocities
        self.left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        self.right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
    
    def update_robot_state(self):
        """Update robot state and publish joint states and odometry"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Update wheel positions
        self.left_wheel_pos += self.left_wheel_vel * dt
        self.right_wheel_pos += self.right_wheel_vel * dt
        
        # Calculate odometry
        left_dist = self.left_wheel_vel * dt * self.wheel_radius
        right_dist = self.right_wheel_vel * dt * self.wheel_radius
        
        # Robot forward distance and rotation
        distance = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_separation
        
        # Update robot pose
        self.x += distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_pub.publish(joint_state)
        
        # Publish transform from base_footprint to odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    controller = DifferentialDriveController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()