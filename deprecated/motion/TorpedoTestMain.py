import numpy as np
import cv2

# cap = cv2.VideoCapture(1)
cap = cv2.VideoCapture("vid2.mp4")

sensitivity1 = 10  # Higher will
sensitivity2 = 100  # Larger is less false positives

param1 = 1
param2 = 325  # Higher is less lines in edges # Lower is More lines in edges


while True:
    # Capture frame-by-frame
    ret, captured_frame = cap.read()
    # output_frame = captured_frame.copy()
    output_frame = captured_frame

    # captured_frame_bgr = cv2.medianBlur(captured_frame_bgr, 3)

    gray = cv2.cvtColor(captured_frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, param1, param2)
    # Blurring may need to be removed
    edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
    edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
    # edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
    # edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)

    circles = cv2.HoughCircles(
        edges,
        cv2.HOUGH_GRADIENT,
        1,
        edges.shape[0] / 8,
        param1=sensitivity1,
        param2=sensitivity2,
        minRadius=1,
        maxRadius=600,
    )

    # If we have extracted a circle, draw an outline
    # Need to paint all circles
    if circles is not None:
        print(circles)
        circles = np.round(circles[0, :]).astype("int")
        i = 0
        for circle in circles:
            cv2.circle(
                output_frame,
                center=(circles[i, 0], circles[i, 1]),
                radius=circles[i, 2],
                color=(0, 255, 0),
                thickness=20,
            )
            cv2.circle(
                output_frame,
                center=(circles[i, 0], circles[i, 1]),
                radius=2,
                color=(0, 0, 255),
                thickness=2,
            )

            # i += 1
    # Display the resulting frame, quit with q
    cv2.imshow("frame", output_frame)
    cv2.imshow("edges", edges)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
