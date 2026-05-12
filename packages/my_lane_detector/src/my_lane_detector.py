#!/usr/bin/env python3

# Python Libs
import sys
import time

# numpy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge

# ROS Libraries
import rospy
import roslib

# ROS Message Types
from sensor_msgs.msg import CompressedImage


class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # Duckiebot camera topic
        self.image_topic = "/mybota002409/camera_node/image/compressed"

        # Tuning parameters
        self.crop_start_y = rospy.get_param("~crop_start_y", 250)
        self.canny_low = rospy.get_param("~canny_low", 50)
        self.canny_high = rospy.get_param("~canny_high", 150)

        self.hough_threshold = rospy.get_param("~hough_threshold", 40)
        self.min_line_length = rospy.get_param("~min_line_length", 30)
        self.max_line_gap = rospy.get_param("~max_line_gap", 20)

        rospy.init_node("my_lane_detector")

        rospy.loginfo("Subscribing to: %s", self.image_topic)

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
        )

    def image_callback(self, msg):

        # Convert ROS image -> OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Print image info once
        if not hasattr(self, "printed_info"):
            rospy.loginfo("Image Shape: %s", img.shape)
            rospy.loginfo("Image Type: %s", img.dtype)
            self.printed_info = True

        ############################################
        # 1. Crop image
        ############################################

        height, width = img.shape[:2]

        crop_start_y = min(max(self.crop_start_y, 0), height - 1)

        cropped = img[crop_start_y:height, 0:width]

        ############################################
        # 2. Convert BGR -> HSV
        ############################################

        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        ############################################
        # 3. White filtering
        ############################################

        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 80, 255])

        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        ############################################
        # 4. Yellow filtering
        ############################################

        lower_yellow = np.array([15, 60, 80])
        upper_yellow = np.array([40, 255, 255])

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        ############################################
        # 5. Morphology cleanup
        ############################################

        kernel = np.ones((5, 5), np.uint8)

        white_mask_clean = cv2.erode(white_mask, kernel, iterations=1)
        white_mask_clean = cv2.dilate(white_mask_clean, kernel, iterations=2)

        yellow_mask_clean = cv2.erode(yellow_mask, kernel, iterations=1)
        yellow_mask_clean = cv2.dilate(yellow_mask_clean, kernel, iterations=2)

        ############################################
        # 6. Canny Edge Detection
        ############################################

        white_edges = cv2.Canny(
            white_mask_clean,
            self.canny_low,
            self.canny_high
        )

        yellow_edges = cv2.Canny(
            yellow_mask_clean,
            self.canny_low,
            self.canny_high
        )

        ############################################
        # 7. Hough Transform (White)
        ############################################

        white_lines = cv2.HoughLinesP(
            white_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )

        ############################################
        # 8. Hough Transform (Yellow)
        ############################################

        yellow_lines = cv2.HoughLinesP(
            yellow_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )

        ############################################
        # 9. Draw lines
        ############################################

        line_img = np.copy(cropped)

        # Blue lines = white lane detection
        line_img = self.output_lines(
            line_img,
            white_lines,
            color=(255, 0, 0)
        )

        # Yellow lines = yellow lane detection
        line_img = self.output_lines(
            line_img,
            yellow_lines,
            color=(0, 255, 255)
        )

        ############################################
        # 10. Put processed crop back
        ############################################

        img_out = np.copy(img)

        img_out[crop_start_y:height, 0:width] = line_img

        ############################################
        # 11. Display windows
        ############################################

        white_filtered = cv2.bitwise_and(
            cropped,
            cropped,
            mask=white_mask_clean
        )

        yellow_filtered = cv2.bitwise_and(
            cropped,
            cropped,
            mask=yellow_mask_clean
        )

        cv2.imshow("white_filtered", white_filtered)

        cv2.imshow("yellow_filtered", yellow_filtered)

        cv2.imshow("hough_lines_output", img_out)

        cv2.waitKey(1)

    def output_lines(self, original_image, lines, color=(255, 0, 0)):

        output = np.copy(original_image)

        if lines is not None:

            for i in range(len(lines)):

                l = lines[i][0]

                x1, y1, x2, y2 = l

                # Ignore nearly horizontal lines
                if abs(y2 - y1) < 10:
                    continue

                cv2.line(
                    output,
                    (x1, y1),
                    (x2, y2),
                    color,
                    2,
                    cv2.LINE_AA
                )

                cv2.circle(output, (x1, y1), 3, (0, 255, 0), -1)

                cv2.circle(output, (x2, y2), 3, (0, 0, 255), -1)

        return output

    def run(self):
        rospy.spin()


if __name__ == "__main__":

    try:

        lane_detector_instance = Lane_Detector()

        lane_detector_instance.run()

    except rospy.ROSInterruptException:
        pass
