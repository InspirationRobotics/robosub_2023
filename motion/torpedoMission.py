import numpy as np
import cv2

cap = cv2.VideoCapture(1)

while(True):
    # Capture frame-by-frame
    ret, captured_frame = cap.read()
    #output_frame = captured_frame.copy()
    output_frame = captured_frame

    # captured_frame_bgr = cv2.medianBlur(captured_frame_bgr, 3)

    gray = cv2.cvtColor(captured_frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 0, 250)
    # Blurring may need to be removed
    edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)

    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, edges.shape[0] / 8, param1=100, param2=60, minRadius=5, maxRadius=60)

	# If we have extracted a circle, draw an outline
	# Need to paint all circles
    if circles is not None:
        print (circles)
        circles = np.round(circles[0, :]).astype("int")
        cv2.circle(output_frame, center=(circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(0, 255, 0), thickness=2)

    # Display the resulting frame, quit with q
    cv2.imshow('frame', output_frame)
    cv2.imshow('edges', edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()