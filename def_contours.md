def getContours(self, inImg, outImg):
    direction = "none"
    contours, _ = cv2.findContours(inImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000 or area > 50000:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        # Arrows often form 7-point approximations
        if len(approx) != 7:
            continue

        # Check convexity: arrows are NOT fully convex
        if cv2.isContourConvex(approx):
            continue

        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        if aspect_ratio < 0.5 or aspect_ratio > 2.5:
            continue

        # All checks passed → likely arrow
        x_vals = approx[:, 0, 0]
        arrow_center = (max(x_vals) + min(x_vals)) / 2
        if np.median(x_vals) < arrow_center:
            direction = "left"
        else:
            direction = "right"

        # Draw & label
        cv2.drawContours(outImg, [approx], -1, (0, 255, 0), 2)
        cv2.rectangle(outImg, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.putText(outImg, direction, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        break  # use only the first confident detection

    return direction, outImg
