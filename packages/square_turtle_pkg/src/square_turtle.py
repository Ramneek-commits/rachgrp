#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_square():
    rospy.init_node('square_turtle_node', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    rospy.loginfo("Turtles are great at drawing squares!")

    while not rospy.is_shutdown():

        # Move forward
        move_msg = Twist()
        move_msg.linear.x = 2.0
        move_msg.angular.z = 0.0

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < 2.0 and not rospy.is_shutdown():
            pub.publish(move_msg)
            rate.sleep()

        # Stop
        stop_msg = Twist()
        pub.publish(stop_msg)
        rospy.sleep(1)

        # Turn 90 degrees
        turn_msg = Twist()
        turn_msg.linear.x = 0.0
        turn_msg.angular.z = 1.57

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < 1.0 and not rospy.is_shutdown():
            pub.publish(turn_msg)
            rate.sleep()

        # Stop again
        pub.publish(stop_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
