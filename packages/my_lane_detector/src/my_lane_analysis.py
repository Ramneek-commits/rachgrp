#!/usr/bin/env python3

import numpy as np
import cv2
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import CompressedImage


class LaneAnalysis:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.image_topic = "/mybota002409/camera_node/image/compressed"

        self.crop_start_y = 150
        self.saved = False

        rospy.init_node("my_lane_analysis")
        rospy.loginfo("Subscribing to: %s", self.image_topic)

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
        )

    def image_callback(self, msg):
        if self.saved:
            return

        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        height, width = img.shape[:2]
        crop_start_y = min(max(self.crop_start_y, 0), height - 1)
        cropped = img[crop_start_y:height, 0:width]

        # Save original cropped frame too
        cv2.imwrite("original_cropped_frame.png", cropped)

        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([15, 60, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        white_mask = cv2.subtract(white_mask, yellow_mask)

        kernel = np.ones((5, 5), np.uint8)

        white_mask = cv2.erode(white_mask, kernel, iterations=1)
        white_mask = cv2.dilate(white_mask, kernel, iterations=2)

        yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=2)

        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # =========================
        # CANNY EXPERIMENTS
        # =========================

        canny_10_100 = cv2.Canny(combined_mask, 10, 100)
        canny_50_150 = cv2.Canny(combined_mask, 50, 150)
        canny_100_200 = cv2.Canny(combined_mask, 100, 200)

        cv2.imwrite("canny_10_100.png", canny_10_100)
        cv2.imwrite("canny_50_150.png", canny_50_150)
        cv2.imwrite("canny_100_200.png", canny_100_200)

        # =========================
        # HOUGH EXPERIMENTS
        # Uses Canny 10-100 as input
        # =========================

        hough_10 = self.hough_experiment(cropped, canny_10_100, 10)
        hough_40 = self.hough_experiment(cropped, canny_10_100, 40)
        hough_70 = self.hough_experiment(cropped, canny_10_100, 70)

        cv2.imwrite("hough_10.png", hough_10)
        cv2.imwrite("hough_40.png", hough_40)
        cv2.imwrite("hough_70.png", hough_70)

        # =========================
        # HSV VS BGR YELLOW
        # =========================

        hsv_yellow = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        lower_yellow_bgr = np.array([0, 120, 120])
        upper_yellow_bgr = np.array([120, 255, 255])
        yellow_bgr_mask = cv2.inRange(cropped, lower_yellow_bgr, upper_yellow_bgr)
        bgr_yellow = cv2.bitwise_and(cropped, cropped, mask=yellow_bgr_mask)

        cv2.imwrite("hsv_yellow.png", hsv_yellow)
        cv2.imwrite("bgr_yellow.png", bgr_yellow)

        # =========================
        # LIGHTING + CANNY 10-100
        # =========================

        dark_img = cv2.convertScaleAbs(cropped, alpha=0.6, beta=-30)
        bright_img = cv2.convertScaleAbs(cropped, alpha=1.3, beta=30)

        canny_10_100_normal = self.canny_from_lighting_image(cropped)
        canny_10_100_dark = self.canny_from_lighting_image(dark_img)
        canny_10_100_bright = self.canny_from_lighting_image(bright_img)

        cv2.imwrite("canny_10_100_normal.png", canny_10_100_normal)
        cv2.imwrite("canny_10_100_dark.png", canny_10_100_dark)
        cv2.imwrite("canny_10_100_bright.png", canny_10_100_bright)

        print("Saved all experiment images.")

        self.saved = True
        rospy.signal_shutdown("Saved all outputs from one frame")

    def hough_experiment(self, image, edges, threshold_value):
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=threshold_value,
            minLineLength=30,
            maxLineGap=20,
        )

        return self.output_lines(image, lines, (255, 0, 0))

    def canny_from_lighting_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([15, 60, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        white_mask = cv2.subtract(white_mask, yellow_mask)
        mask = cv2.bitwise_or(white_mask, yellow_mask)

        return cv2.Canny(mask, 10, 100)

    def output_lines(self, original_image, lines, color=(255, 0, 0)):
        output = np.copy(original_image)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                if abs(y2 - y1) < 10:
                    continue

                cv2.line(output, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
                cv2.circle(output, (x1, y1), 3, (0, 255, 0), -1)
                cv2.circle(output, (x2, y2), 3, (0, 0, 255), -1)

        return output

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        print("Starting lane analysis node...")
        node = LaneAnalysis()
        print("Node created. Waiting for one image message...")
        node.run()
    except rospy.ROSInterruptException:
        pass
