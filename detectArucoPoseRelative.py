from multiprocessing.connection import wait
from tkinter import Frame
import numpy as np
import cv2
import time
from imutils.video import VideoStream
import imutils
import math
from scipy.spatial.transform import Rotation as R

import utils.utils as utils

# Starting the Camera
vs = VideoStream(src=0).start()
time.sleep(2.0)

# Loading up the parameters obtained in the previous camera calibration
cv_file = cv2.FileStorage(
    'calibration_chessboard_unity.yaml', cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

aruco_marker_side_length = 1 #0.08

# We initialize a dictionary to save the poses, a placeholder world origin, and the ArUco dictionary 
path = '.'
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
params = cv2.aruco.DetectorParameters_create()
Poses = {}
origin = -1
toy = 1
while True:
    # Loading the image / camera frame
    #frame = vs.read()
    path = './examples/'
    
    frame = cv2.imread(path + 'toy3.PNG')

    #frame = imutils.resize(frame, height=1920, width=1080)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, dict, parameters=params, cameraMatrix=mtx, distCoeff=dst)

    utils.drawMarkerFeatures(frame, rejected)

    if len(corners) > 0:
        # If we don't have a world origin, we get the first marker as it
        # Si no tenemos un origen de coordenadas, tomamos el primer marcador como tal
        if origin == -1:
            origin = ids[0][0]
            print("Origen establecido en {}".format(origin))

        # Rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_marker_side_length, mtx, dst)
        # tvecs: Translation vector in 3D
        # rvecs: Rotation vector

        for i, marker_id in enumerate(ids):
            if not marker_id[0] == origin:
                #print("Looking into", marker_id[0])
                if not marker_id[0] in Poses:   
                    j = 0
                    Found = False
                    while j < len(ids) - 1 and not Found:
                        print("J: {}, Id: {}, Origin: {}".format(j, ids[j][0], origin))
                        if ids[j][0] == origin or ids[j][0] in Poses:
                            Found = True
                        else:
                            j = j + 1

                    # Did we find something? What?
                    if ids[j][0] == origin:
                        # We found the world origin, so we can calculate the pose from the marker directly
                        Poses[ids[i][0]] = utils.calcPoseDirectly(tvecs, rvecs, i, j, marker_id, Debug=True)
                        print("Direct")
                        print(ids[i][0])
                        print(Poses[ids[i][0]])
                        break
                    elif ids[j][0] in Poses:
                        # We found another marker with its pose relative to the origin already calculated. We can concatenate transformations to get the pose we need
                        Poses[ids[i][0]] = utils.calcPoseIndirectly(tvecs, rvecs, i, j, Poses, ids, Debug=True)
                        print("Indirect")
                        print(ids[i][0])
                        print(Poses[ids[i][0]])
                        break
                    else:
                        #No hemos encontrado nada, mensaje de error
                        break
        
    
# Display the resulting frame
    cv2.imshow("Camara", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop() 

