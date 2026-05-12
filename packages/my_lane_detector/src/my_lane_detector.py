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

        # Your Duckiebot name. You said your bot name is mybota002409.
        self.image_topic = "/mybota002409/camera_node/image/compressed"

        # Tune these values if the output is too noisy or misses the lane lines.
        self.crop_start_y = rospy.get_param("~crop_start_y", 250)
        self.canny_low = rospy.get_param("~canny_low", 50)
        self.canny_high = rospy.get_param("~canny_high", 150)
        self.hough_threshold = rospy.get_param("~hough_threshold", 40)
        self.min_line_length = rospy.get_param("~min_line_length", 30)
        self.max_line_gap = rospy.get_param("~max_line_gap", 20)

        rospy.init_node("my_lane_detector")
        rospy.loginfo("Subscribing to image topic: %s", self.image_topic)

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
        )

    def image_callback(self, msg):
        # Convert ROS compressed image message to OpenCV BGR image.
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Only print image info once, not every frame.
        if not hasattr(self, "printed_info"):
            rospy.loginfo("Image type: %s", type(img))
            rospy.loginfo("Image dimensions: %s", img.ndim)
            rospy.loginfo("Image shape: %s", img.shape)
            rospy.loginfo("Image size: %s", img.size)
            rospy.loginfo("Image data type: %s", img.dtype)
            self.printed_info = True

        #### IMAGE PROCESSING PIPELINE ####

        # 1. Crop the lower part of the image because the road/lane is usually there.
        height, width = img.shape[:2]
        crop_start_y = min(max(self.crop_start_y, 0), height - 1)
        cropped = img[crop_start_y:height, 0:width]

        # 2. Convert to HSV. HSV makes colour filtering easier than raw BGR.
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # 3. Filter likely white lane markings.
        # White usually has low saturation and high value.
        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 80, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # 4. Filter likely yellow lane markings.
        # You may need to tune this depending on lighting.
        lower_yellow = np.array([15, 60, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 5. Combine white and yellow masks.
        mask = cv2.bitwise_or(white_mask, yellow_mask)

        # 6. Clean up small noise using morphology.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # 7. Find edges from the mask.
        edges = cv2.Canny(mask, self.canny_low, self.canny_high)

        # 8. Detect line segments using probabilistic Hough transform.
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )

        # 9. Draw detected lines on the cropped image.
        line_img = self.output_lines(cropped, lines)

        # 10. Put the processed crop back into the full image for easier viewing.
        img_out = np.copy(img)
        img_out[crop_start_y:height, 0:width] = line_img

        #############################

        # Show images in windows.
        cv2.imshow("lane_detection_output", img_out)
        cv2.imshow("lane_mask", mask)
        cv2.imshow("lane_edges", edges)
        cv2.waitKey(1)

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)

        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                x1, y1, x2, y2 = l

                # Ignore almost-horizontal lines because lane markers are usually angled/vertical in the image.
                if abs(y2 - y1) < 10:
                    continue

                cv2.line(output, (x1, y1), (x2, y2), (255, 0, 0), 2, cv2.LINE_AA)
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
