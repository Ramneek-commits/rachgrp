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

        # Change this for each experiment:
        # canny_30_100
        # canny_50_150
        # canny_100_200
        # hough_20
        # hough_40
        # hough_70
        # hsv_yellow
        # bgr_yellow
        # lighting_normal
        # lighting_dark
        # lighting_bright
        self.experiment = "canny_50_150"

        rospy.init_node("my_lane_analysis")
        rospy.loginfo("Subscribing to: %s", self.image_topic)
        rospy.loginfo("Running experiment: %s", self.experiment)

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

        output = cropped

        if self.experiment == "canny_30_100":
            output = cv2.Canny(combined_mask, 30, 100)

        elif self.experiment == "canny_50_150":
            output = cv2.Canny(combined_mask, 50, 150)

        elif self.experiment == "canny_100_200":
            output = cv2.Canny(combined_mask, 100, 200)

        elif self.experiment == "hough_20":
            output = self.hough_experiment(cropped, combined_mask, 20)

        elif self.experiment == "hough_40":
            output = self.hough_experiment(cropped, combined_mask, 40)

        elif self.experiment == "hough_70":
            output = self.hough_experiment(cropped, combined_mask, 70)

        elif self.experiment == "hsv_yellow":
            output = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        elif self.experiment == "bgr_yellow":
            lower_yellow_bgr = np.array([0, 120, 120])
            upper_yellow_bgr = np.array([120, 255, 255])
            yellow_bgr_mask = cv2.inRange(cropped, lower_yellow_bgr, upper_yellow_bgr)
            output = cv2.bitwise_and(cropped, cropped, mask=yellow_bgr_mask)

        elif self.experiment == "lighting_normal":
            output = self.run_lane_detection_for_lighting(cropped)

        elif self.experiment == "lighting_dark":
            dark_img = cv2.convertScaleAbs(cropped, alpha=0.6, beta=-30)
            output = self.run_lane_detection_for_lighting(dark_img)

        elif self.experiment == "lighting_bright":
            bright_img = cv2.convertScaleAbs(cropped, alpha=1.3, beta=30)
            output = self.run_lane_detection_for_lighting(bright_img)

        else:
            rospy.logwarn("Unknown experiment: %s", self.experiment)

        filename = self.experiment + ".png"
        cv2.imwrite(filename, output)
        print("Saved:", filename)

        self.saved = True
        rospy.signal_shutdown("Saved one processed frame")

    def hough_experiment(self, image, mask, threshold_value):
        edges = cv2.Canny(mask, 50, 150)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=threshold_value,
            minLineLength=30,
            maxLineGap=20,
        )

        return self.output_lines(image, lines, (255, 0, 0))

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
        print("Starting lane analysis node...")
        node = LaneAnalysis()
        print("Node created. Waiting for one image message...")
        node.run()
    except rospy.ROSInterruptException:
        pass
