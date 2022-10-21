from matplotlib import image
import numpy as np
import cv2

def drawMarkerFeatures(img, corners, ids):
    ids = ids.flatten()
    for(marker, id) in zip(corners, ids):
        corners = marker.reshape(4, 2)
        (topLeft, topRight, botRight, botLeft) = corners
        
        # Marcamos las esquinas y el centro de color rojo en la imagen
        cv2.circle(img, topLeft.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, topRight.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, botRight.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, botLeft.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, (int((topLeft[0] + botRight[0])/2), int((topLeft[1] + botRight[1])/2)), 1, (0,0,255), -1)

        # Marcamos el contorno con líneas verdes
        cv2.line(img, topLeft.astype(int), topRight.astype(int), (255, 0, 0), 2)
        cv2.line(img, topLeft.astype(int), botLeft.astype(int), (255, 0, 0), 2)
        cv2.line(img, botLeft.astype(int), botRight.astype(int), (255, 0, 0), 2)
        cv2.line(img, botRight.astype(int), topRight.astype(int), (255, 0, 0), 2)

        # Indicamos la ID del marker
        cv2.putText(img, str(id), (int(topLeft[0]), int(topLeft[1]-7)), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 1)


path = './markers/'
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
params = cv2.aruco.DetectorParameters_create()
img = cv2.imread('markers_detect.jpg')
(corners, ids, rejected) = cv2.aruco.detectMarkers(img, dict, parameters=params)

if len(corners) > 0:
    ids = ids.flatten()
    drawMarkerFeatures(img, corners, ids)

    # Corners representa la imagen como una lista de códigos ArUco. Cada código ArUco está representado como una lista de cuatro puntos
    # con coordenada X e Y. Los puntos están en orden de arriba izquierda a abajo derecha
            

cv2.imwrite('markers-detected.png', img)
cv2.imshow("Image", img)
cv2.waitKey(0)

