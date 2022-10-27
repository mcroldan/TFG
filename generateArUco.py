import numpy as np
import cv2

path = './markers/'
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
marker = np.zeros((100, 100, 1), dtype="uint8")

# Generate 100 markers

for i in range(0, 100, 1):
    cv2.aruco.drawMarker(dict, i, 100, marker, 1)

    cv2.imwrite(path + 'marker-' + str(i) + '.png', marker)




