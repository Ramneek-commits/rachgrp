#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        self.cmd_vel_pub = rospy.Publisher('/mybota002409/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/mybota002409/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):

        if len(detections) == 0:
            rospy.loginfo("No tag detected. Stopping...")
            self.stop_robot()
            return

        x = detections[0].transform.translation.x
        y = detections[0].transform.translation.y
        z = detections[0].transform.translation.z

        rospy.loginfo("x,y,z: %f %f %f", x, y, z)

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        # -----------------------------
        # Calibration values
        # -----------------------------
        target_distance = 0.4   # desired distance from tag 
        kp_turn = 3.0           # turning strength
        kp_forward = 0.8        # forward/backward strength

        max_omega = 2.0         # max turning speed
        max_v = 0.3             # max forward speed

        x_deadzone = 0.02       # stop turning if tag is nearly centered
        z_deadzone = 0.05       # stop moving if distance is close enough


        if abs(x) < x_deadzone:
            omega = 0.0
        else:
            omega = -kp_turn * x

        # Limit omega
        if omega > max_omega:
            omega = max_omega
        if omega < -max_omega:
            omega = -max_omega


        distance_error = z - target_distance

        if abs(distance_error) < z_deadzone:
            v = 0.0
        else:
            v = kp_forward * distance_error

        # Limit forward/backward speed
        if v > max_v:
            v = max_v
        if v < -max_v:
            v = -max_v


        cmd_msg.v = v
        cmd_msg.omega = omega

        self.cmd_vel_pub.publish(cmd_msg)
if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
