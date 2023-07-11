import cv2


# --- First obtain the threshold using the greyscale image ---
cap = cv2.VideoCapture("vid6.mp4")

while True:
    ret, captured_frame = cap.read()
    img = captured_frame

    gray = cv2.cvtColor(captured_frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 0, 400)
    # Blurring may need to be removed
    edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
    edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
    edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
    edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)

    # ret,th = cv2.threshold(gray,127,255, 0)

    # --- Find all the contours in the binary image ---
    contours, hierarchy = cv2.findContours(edges, 2, 1)
    cnt = contours
    big_contour = []
    big_contour2 = []

    max = 0
    max2 = 0
    for pic in cnt:
        area = cv2.contourArea(pic)  # --- find the contour having biggest area ---
        if area > max:
            max = area
            big_contour = pic
        elif (area < max) and (area > max2):
            max2 = area
            big_contour2 = pic

    final = cv2.drawContours(img, big_contour, -1, (0, 255, 0), 3)
    final = cv2.drawContours(img, big_contour2, -1, (0, 255, 0), 3)

    cv2.imshow("final", final)
    cv2.imshow("edges", edges)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
