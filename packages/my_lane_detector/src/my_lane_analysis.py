def image_callback(self, msg):

    img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

    height, width = img.shape[:2]

    cropped = img[250:height, 0:width]

    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

    # ============================================
    # HSV FILTERING
    # ============================================

    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 40, 255])

    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    lower_yellow = np.array([15, 60, 80])
    upper_yellow = np.array([40, 255, 255])

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    white_mask = cv2.subtract(white_mask, yellow_mask)

    mask = cv2.bitwise_or(white_mask, yellow_mask)

    # ============================================
    # CHOOSE ONE EXPERIMENT ONLY
    # ============================================

    # ---------- CANNY ----------
    output = cv2.Canny(mask, 50, 150)

    # ---------- HOUGH ----------
    """
    edges = cv2.Canny(mask, 50, 150)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=30,
        maxLineGap=20,
    )

    output = self.output_lines(cropped, lines)
    """

    # ---------- HSV FILTER ----------
    """
    output = cv2.bitwise_and(
        cropped,
        cropped,
        mask=yellow_mask
    )
    """

    # ---------- LIGHTING TEST ----------
    """
    dark_img = cv2.convertScaleAbs(
        cropped,
        alpha=0.6,
        beta=-30
    )

    output = dark_img
    """

    cv2.imshow("experiment_output", output)

    cv2.waitKey(1)
