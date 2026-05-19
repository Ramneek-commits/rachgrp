#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range


class ClosedLoopSquareCollisionPrevention:
    def __init__(self):
        rospy.init_node("closed_loop_square_collision_node", anonymous=True)

        self.vehicle_name = rospy.get_param("~veh", "mybota002409")

        self.cmd_topic = "/" + self.vehicle_name + "/car_cmd_switch_node/cmd"
        self.fsm_topic = "/" + self.vehicle_name + "/fsm_node/mode"
        self.left_encoder_topic = "/" + self.vehicle_name + "/left_wheel_encoder_node/tick"
        self.right_encoder_topic = "/" + self.vehicle_name + "/right_wheel_encoder_node/tick"
        self.tof_topic = "/" + self.vehicle_name + "/front_center_tof_driver_node/range"

        self.pub = rospy.Publisher(self.cmd_topic, Twist2DStamped, queue_size=1)

        rospy.Subscriber(self.fsm_topic, FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber(self.left_encoder_topic, WheelEncoderStamped, self.left_encoder_callback, queue_size=1)
        rospy.Subscriber(self.right_encoder_topic, WheelEncoderStamped, self.right_encoder_callback, queue_size=1)
        rospy.Subscriber(self.tof_topic, Range, self.tof_callback, queue_size=1)

        self.cmd_msg = Twist2DStamped()

        self.left_ticks = 0
        self.right_ticks = 0
        self.start_left_ticks = 0
        self.start_right_ticks = 0

        self.left_ready = False
        self.right_ready = False

        self.is_running = False

        # Tune these after testing
        self.ticks_per_meter = 850
        self.ticks_per_90_degrees = 40

        # ToF obstacle threshold in metres
        self.stop_distance = 0.25
        self.obstacle_detected = False
        self.latest_tof_distance = None

        rospy.loginfo("Closed loop square collision-prevention node started")
        rospy.loginfo("Command topic: %s", self.cmd_topic)
        rospy.loginfo("ToF topic: %s", self.tof_topic)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data
        self.left_ready = True

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data
        self.right_ready = True

    def tof_callback(self, msg):
        self.latest_tof_distance = msg.range

        if msg.range < self.stop_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def fsm_callback(self, msg):
        rospy.loginfo("FSM State: %s", msg.state)

        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
            self.is_running = False

        elif msg.state == "LANE_FOLLOWING":
            if not self.is_running:
                self.is_running = True
                rospy.sleep(1)
                self.run_demo_sequence()
                self.is_running = False

    def publish_cmd(self, v, omega):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = v
        self.cmd_msg.omega = omega
        self.pub.publish(self.cmd_msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)
        rospy.sleep(0.2)

    def wait_for_encoders(self):
        rospy.loginfo("Waiting for encoder messages...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and not (self.left_ready and self.right_ready):
            rate.sleep()

        rospy.loginfo("Encoder messages received")

    def reset_encoder_reference(self):
        self.start_left_ticks = self.left_ticks
        self.start_right_ticks = self.right_ticks

    def average_tick_change(self):
        left_change = abs(self.left_ticks - self.start_left_ticks)
        right_change = abs(self.right_ticks - self.start_right_ticks)
        return (left_change + right_change) / 2.0

    def wait_until_obstacle_removed(self, rate):
        rospy.loginfo("Obstacle detected. Stopping and waiting...")

        self.stop_robot()

        while not rospy.is_shutdown() and self.obstacle_detected:
            self.publish_cmd(0.0, 0.0)
            rate.sleep()

        rospy.loginfo("Obstacle removed. Continuing mission...")

    def move_straight(self, distance_m, speed):
        self.wait_for_encoders()
        self.reset_encoder_reference()

        target_ticks = abs(distance_m) * self.ticks_per_meter

        if distance_m >= 0:
            v = abs(speed)
        else:
            v = -abs(speed)

        rospy.loginfo("Moving %.2f m at speed %.2f", distance_m, v)
        rospy.loginfo("Target ticks: %.2f", target_ticks)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            current_ticks = self.average_tick_change()

            if current_ticks >= target_ticks:
                break

            # Collision prevention ONLY during forward straight movement
            if distance_m > 0 and self.obstacle_detected:
                self.wait_until_obstacle_removed(rate)
                self.reset_encoder_reference()

                # Important:
                # Because we reset encoder reference after waiting,
                # reduce remaining target distance by recalculating remaining ticks.
                remaining_ticks = target_ticks - current_ticks
                target_ticks = max(remaining_ticks, 0)

            self.publish_cmd(v, 0.0)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Straight completed. Final ticks: %.2f", self.average_tick_change())

    def rotate_in_place(self, angle_degrees, angular_speed):
        self.wait_for_encoders()
        self.reset_encoder_reference()

        target_ticks = abs(angle_degrees) / 90.0 * self.ticks_per_90_degrees

        if angle_degrees >= 0:
            omega = abs(angular_speed)
        else:
            omega = -abs(angular_speed)

        rospy.loginfo("Rotating %.2f degrees at omega %.2f", angle_degrees, omega)
        rospy.loginfo("Target rotation ticks: %.2f", target_ticks)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            current_ticks = self.average_tick_change()

            if current_ticks >= target_ticks:
                break

            # Do NOT check obstacle here.
            # Collision prevention should not affect in-place rotation.
            self.publish_cmd(0.0, omega)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Rotation completed. Final ticks: %.2f", self.average_tick_change())

    def draw_square(self):
        rospy.loginfo("Starting closed loop square with collision prevention")

        for i in range(4):
            rospy.loginfo("Square side %d", i + 1)
            self.move_straight(1.0, 0.30)

            rospy.loginfo("Square turn %d", i + 1)
            self.rotate_in_place(90, 3.5)

        self.stop_robot()
        rospy.loginfo("Square completed")

    def run_demo_sequence(self):
        self.draw_square()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ClosedLoopSquareCollisionPrevention()
        node.run()
    except rospy.ROSInterruptException:
        pass
