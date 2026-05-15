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

        rospy.init_node("my_lane_analysis")
        rospy.loginfo("Subscribing to: %s", self.image_topic)

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
        )

    def image_callback(self, msg):
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        height, width = img.shape[:2]
        crop_start_y = min(max(self.crop_start_y, 0), height - 1)
        cropped = img[crop_start_y:height, 0:width]

        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # -----------------------------
        # White and yellow HSV masks
        # -----------------------------
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([15, 60, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Remove yellow leakage from white
        white_mask = cv2.subtract(white_mask, yellow_mask)

        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.erode(white_mask, kernel, iterations=1)
        white_mask = cv2.dilate(white_mask, kernel, iterations=2)

        yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=2)

        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # ==================================================
        # 1. CANNY EXPERIMENTS: use 3 threshold pairs
        # ==================================================
        canny_30_100 = cv2.Canny(combined_mask, 30, 100)
        canny_50_150 = cv2.Canny(combined_mask, 50, 150)
        canny_100_200 = cv2.Canny(combined_mask, 100, 200)

        cv2.imshow("Canny 30 100", canny_30_100)
        cv2.imshow("Canny 50 150", canny_50_150)
        cv2.imshow("Canny 100 200", canny_100_200)

        # ==================================================
        # 2. HOUGH EXPERIMENTS: change threshold only
        # ==================================================
        hough_input = canny_50_150

        lines_20 = cv2.HoughLinesP(
            hough_input,
            rho=1,
            theta=np.pi / 180,
            threshold=20,
            minLineLength=30,
            maxLineGap=20,
        )

        lines_40 = cv2.HoughLinesP(
            hough_input,
            rho=1,
            theta=np.pi / 180,
            threshold=40,
            minLineLength=30,
            maxLineGap=20,
        )

        lines_70 = cv2.HoughLinesP(
            hough_input,
            rho=1,
            theta=np.pi / 180,
            threshold=70,
            minLineLength=30,
            maxLineGap=20,
        )

        hough_20_img = self.output_lines(cropped, lines_20, (255, 0, 0))
        hough_40_img = self.output_lines(cropped, lines_40, (0, 255, 0))
        hough_70_img = self.output_lines(cropped, lines_70, (0, 0, 255))

        cv2.imshow("Hough threshold 20", hough_20_img)
        cv2.imshow("Hough threshold 40", hough_40_img)
        cv2.imshow("Hough threshold 70", hough_70_img)

        # ==================================================
        # 3. HSV vs BGR/RGB yellow detection
        # ==================================================
        hsv_yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # OpenCV image is BGR, not RGB.
        # This roughly catches yellow in BGR format.
        lower_yellow_bgr = np.array([0, 120, 120])
        upper_yellow_bgr = np.array([120, 255, 255])
        yellow_bgr_mask = cv2.inRange(cropped, lower_yellow_bgr, upper_yellow_bgr)
        bgr_yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_bgr_mask)

        cv2.imshow("HSV yellow filter", hsv_yellow_result)
        cv2.imshow("BGR yellow filter", bgr_yellow_result)

        # ==================================================
        # 4. Lighting condition experiment
        # ==================================================
        dark_img = cv2.convertScaleAbs(cropped, alpha=0.6, beta=-30)
        bright_img = cv2.convertScaleAbs(cropped, alpha=1.3, beta=30)

        normal_lines_img = self.run_lane_detection_for_lighting(cropped)
        dark_lines_img = self.run_lane_detection_for_lighting(dark_img)
        bright_lines_img = self.run_lane_detection_for_lighting(bright_img)

        cv2.imshow("Lighting normal", normal_lines_img)
        cv2.imshow("Lighting dark", dark_lines_img)
        cv2.imshow("Lighting bright", bright_lines_img)

        cv2.waitKey(1)

    def run_lane_detection_for_lighting(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([15, 60, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        white_mask = cv2.subtract(white_mask, yellow_mask)

        mask = cv2.bitwise_or(white_mask, yellow_mask)

        edges = cv2.Canny(mask, 50, 150)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=40,
            minLineLength=30,
            maxLineGap=20,
        )

        return self.output_lines(image, lines, (255, 0, 0))

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
        node = LaneAnalysis()
        node.run()
    except rospy.ROSInterruptException:
        pass
