#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose

class TurtlesimStraightsAndTurns:
    def __init__(self):
        self.last_distance = 0.0
        self.start_distance = 0.0
        self.goal_distance = 0.0
        self.dist_goal_active = False
        self.forward_movement = True

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.goal_angle = 0.0
        self.angle_goal_active = False
        self.target_theta = 0.0
        self.rotate_ccw = True

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.position_goal_active = False

        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        rospy.loginfo("Initialized node!")
        rospy.spin()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_angle_callback(self, msg):
        if msg.data == 0:
            self.angle_goal_active = False
            return

        self.goal_angle = abs(msg.data)
        self.rotate_ccw = msg.data > 0

        if self.rotate_ccw:
            self.target_theta = self.normalize_angle(self.current_theta + self.goal_angle)
        else:
            self.target_theta = self.normalize_angle(self.current_theta - self.goal_angle)

        self.angle_goal_active = True
        self.dist_goal_active = False
        self.position_goal_active = False

    def goal_distance_callback(self, msg):
        if msg.data == 0:
            self.goal_distance = 0
            self.dist_goal_active = False
            return

        self.goal_distance = abs(msg.data)
        self.start_distance = self.last_distance
        self.dist_goal_active = True
        self.forward_movement = msg.data > 0
        self.angle_goal_active = False
        self.position_goal_active = False

    def goal_position_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.position_goal_active = True
        self.dist_goal_active = False
        self.angle_goal_active = False

    def timer_callback(self, event):
        cmd = Twist()

        if self.dist_goal_active:
            travelled = abs(self.last_distance - self.start_distance)

            if travelled >= self.goal_distance - 0.05:
                self.dist_goal_active = False
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 1.0 if self.forward_movement else -1.0
                cmd.angular.z = 0.0

        elif self.angle_goal_active:
            error = self.normalize_angle(self.target_theta - self.current_theta)

            if abs(error) < 0.03:
                self.angle_goal_active = False
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.0 if error > 0 else -1.0

        elif self.position_goal_active:
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            distance_to_goal = math.sqrt(dx**2 + dy**2)
            desired_theta = math.atan2(dy, dx)
            angle_error = self.normalize_angle(desired_theta - self.current_theta)

            if distance_to_goal < 0.1:
                self.position_goal_active = False
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                if abs(angle_error) > 0.05:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.0 if angle_error > 0 else -1.0
                else:
                    cmd.linear.x = 1.0
                    cmd.angular.z = 0.0

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.velocity_publisher.publish(cmd)

if __name__ == '__main__':
    try:
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass
