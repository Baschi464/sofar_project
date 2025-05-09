#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# -------------------------------------------------------------- #


class Gotogoal(Node):

    def __init__(self):
        super().__init__('gotogoal')
        
        self.subscription = self.create_subscription(
            Odometry,
            'odom2',
            self.callback_navigate,
            10)
        self.subscription  # prevent unused variable warning

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.callback_scan,
            10)
        self.scan_subscription
       
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)


        # State variables and thresholds
        self.obstacle_detected = False
        self.front_clearance = 0.5  # [m]
        self.side_clearance = 0.5  # [m]
        self.desired_angle_window = 0.3 # [rad] = 17.2[deg] angle window for front scan and right scan
        self.max_angular_speed = 1.5 # [rad/s]

        # State for wall-following
        # if the robot finds a wall in front, it will turn left
        # and keep the wall to its right
        self.following_wall = False  
  

        # Goals 
        self.goals = [(1.0, 1.0), (3.0, 1.0), (5.0, 2.0), (5.0, 5.0), (2.0, 4.5)]
        self.current_goal_index = None

# -------------------------------------------------------------- #

    def compute_wall_following(self, out_msg):

        out_msg.linear.x = 0.0
        out_msg.angular.z = self.max_angular_speed

        return out_msg

# -------------------------------------------------------------- #

    def compute_goal_following(self, out_msg, distance_to_goal, angle_diff):

        # Proportional control for goal-seeking
        Kp_lin = 0.5
        Kp_ang = 1.0
        linear_x = Kp_lin * distance_to_goal
        angular_z = Kp_ang * angle_diff

        angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)  # max angular speed
        linear_x = np.clip(linear_x, 0.0, 0.22)    # max linear speed

        out_msg.linear.x = linear_x
        out_msg.angular.z = angular_z

        return out_msg


# -------------------------------------------------------------- #

    def closest_goal(self, goals, current_x, current_y):
        
        if not goals:
            return None

        # Find the closest goal
        closest_distance = float('inf')
        goal_index = None

        for i, (goal_x, goal_y) in enumerate(goals):
            distance = np.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
            if distance < closest_distance:
                closest_distance = distance
                goal_index = i

        return goal_index


# -------------------------------------------------------------- #
        
    def callback_navigate(self, in_msg):
        out_msg = Twist()

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

        # Goal
        goal = self.closest_goal(self.goals, current_x, current_y)  # closest goal index
        if goal is None:
            goal_x, goal_y = current_x, current_y
            self.get_logger().info("All goals reached! Stopping.")
        else:
            goal_x, goal_y = self.goals[goal]  # current goal coordinates

        # Calculate the distance to the goal
        distance_to_goal = np.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        # Calculate the angle to the goal
        angle_to_goal = np.arctan2(goal_y - current_y, goal_x - current_x)
        # Calculate the angle difference                    
        angle_diff = angle_to_goal - current_yaw
        # Normalize the angle difference to the range [-pi, pi]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))


        # Navgation mode switching
        if not self.following_wall and self.obstacle_detected:
            self.following_wall = True
            self.get_logger().info("Obstacle detected: switching to wall-following.")

        elif self.following_wall and not self.obstacle_detected:
            self.following_wall = False
            self.get_logger().info("Path to goal clear: resuming goal-seeking.")


        # Create a Twist message for velocity commands
        if distance_to_goal < 0.1:
            # Stop the robot
            out_msg.linear.x = 0.0
            out_msg.angular.z = 0.0
            self.get_logger().info("Goal reached! I will go to the next goal.")
            self.goals.pop(goal) # remove the goal from the list
            goal = self.closest_goal(self.goals, current_x, current_y) # switch to the next goal

        elif self.following_wall == True:
            out_msg = self.compute_wall_following(out_msg)

        else: 
            out_msg = self.compute_goal_following(out_msg, distance_to_goal, angle_diff)

        # Publish the Twist message
        self.publisher.publish(out_msg)
            
# -------------------------------------------------------------- #

    def callback_scan(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges = np.clip(ranges, scan_msg.range_min, scan_msg.range_max)

        num_rays = len(ranges)

        # Define angle windows 
        # angle_min = 0.0
        # angle_max = 6.28 
        # angle_increment = 0.0174533 it is positive so it means lidar rotates counter-clockwise
        idx_front = round((0.0 - scan_msg.angle_min)/scan_msg.angle_increment)   # = 0
        idx_right = round((-np.pi/2 - scan_msg.angle_min)/scan_msg.angle_increment) # = -90 
        window = round(self.desired_angle_window/scan_msg.angle_increment) # = 17
        
        front_indices_1 = slice(idx_front, idx_front+window+1) # [0, 17]
        front_indices_2 = slice(idx_front-window+num_rays, idx_front+num_rays) # [-17, 0]+360 = [343, 359]
        right_indices = slice(idx_right-window+num_rays, idx_right+window+num_rays+1) # [-107, -74] + 360 = [253, 288] 

        # Compute stats
        front_dist_1 = np.min(ranges[front_indices_1])  # uses MIN to detect most dangerous obstacle
        front_dist_2 = np.min(ranges[front_indices_2])  # uses MIN to detect most dangerous obstacle
        front_dist = np.min([front_dist_1, front_dist_2])
        right_dist = np.mean(ranges[right_indices]) # uses MEAN to detect the wall distance

        # Update state variables
        self.obstacle_detected = front_dist < self.front_clearance
        self.obstacle_on_right = right_dist < self.side_clearance

    

# -------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    gotogoal = Gotogoal()
    rclpy.spin(gotogoal)
    gotogoal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
