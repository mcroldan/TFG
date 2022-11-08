import numpy as np
import cv2

# In order to detect and remove outliers, I am going to use RANSAC. However, we only have the four corners of the ArUco Markers detected, and we need more than 4 points to
# use RANSAC effectively. That said, I will use Harris corner detector to get more feature points.

# In order to use Harris, I need to solve the homography firt.
# I will only use this function with squares, so the ratio on the dst points will always be 1
def solveDistortedImage(img, corners):
    for marker in corners:
        image = img
        pts_src = np.array([marker[0][0], marker[0][3], marker[0][1], marker[0][2]])
        pts_dst = np.array([[250, 250], [250, 500], [500, 250], [500, 500]]) # Square of ratio 1

        M, _ = cv2.findHomography(pts_src, pts_dst)

        no_perspective = cv2.warpPerspective(image, M, (700, 700))

    return no_perspective[200:550, 200:550]

def applyHarrisToUndistortedImg(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Size of Sobel kernel
    size_sobel = 3

    # Empiric constant K
    const_k = 0.04

    # Size of Gaussian window (in openCV neighbor averaging)
    size_window = 3

    # Apply Harris
    harris = cv2.cornerHarris(gray, size_window, size_sobel, const_k)
    
    # Apply threshold to know whether we have a corner or not
    threshold = .07 * harris.max()
    _, harris_corners = cv2.threshold(harris, threshold, 255, cv2.THRESH_BINARY)
    print(harris_corners)
    return harris_corners
