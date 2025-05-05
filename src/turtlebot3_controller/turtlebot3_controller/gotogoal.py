#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Gotogoal(Node):

    def __init__(self):
        super().__init__('gotogoal')
        
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.callback_navigate,
            10)
        self.subscription  # prevent unused variable warning
       
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def callback_navigate(self, in_msg):

        goal_x = 5.0
        goal_y = 5.0

        # Extract position and orientation from the Odometry message
        position = in_msg.pose.pose.position
        orientation = in_msg.pose.pose.orientation

        # Convert quaternion to Euler angles (yaw)
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)      # sin(yaw)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)  # cos(yaw)
        yaw = np.arctan2(siny_cosp, cosy_cosp)   # yaw angle

        # Current position and orientation
        current_x = position.x
        current_y = position.y
        current_yaw = yaw

        # Calculate the distance to the goal
        distance_to_goal = np.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        # Calculate the angle to the goal
        angle_to_goal = np.arctan2(goal_y - current_y, goal_x - current_x)
        # Calculate the angle difference                    
        angle_diff = angle_to_goal - current_yaw
        # Normalize the angle difference to the range [-pi, pi]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

        # Proportional control for linear and angular velocities
        Kp_lin = 0.5   # Proportional gain for linear velocity
        Kp_ang = 1.0   # Proportional gain for angular velocity
        linear_x = Kp_lin * distance_to_goal
        angular_z = Kp_ang * angle_diff

        # Limit the linear and angular velocities
        angular_z = np.clip(angular_z, -1.5, 1.5)
        linear_x = np.clip(linear_x, 0.0, 0.22)  # max TurtleBot3 speed

        # Create a Twist message to send the velocities
        out_msg = Twist()
        out_msg.linear.x = linear_x
        out_msg.linear.y = 0.0
        out_msg.linear.z = 0.0
        out_msg.angular.x = 0.0
        out_msg.angular.y = 0.0
        out_msg.angular.z = angular_z

        if distance_to_goal < 0.1:
            self.get_logger().info("Goal reached!")
            out_msg.linear.x = 0.0
            out_msg.angular.z = 0.0

        # Publish the Twist message
        self.publisher.publish(out_msg)
            
            
def main(args=None):
    rclpy.init(args=args)
    gotogoal = Gotogoal()
    rclpy.spin(gotogoal)
    gotogoal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
