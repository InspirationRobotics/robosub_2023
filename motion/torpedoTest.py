import numpy as np
import cv2 as cv




img = cv.imread('testPhot.jpeg', cv.IMREAD_GRAYSCALE)
# img = cv.GaussianBlur(img, (5, 5), 2, 2)

assert img is not None, "file could not be read, check with os.path.exists()"
edges = cv.Canny(img,0,250)
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()