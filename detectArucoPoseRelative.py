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

# Vamos preparando la web cam
vs = VideoStream(src=0).start()
time.sleep(2.0)

# Cargamos los parametros de la camara obtenidos en la calibración previamente
cv_file = cv2.FileStorage(
    'calibration_chessboard.yaml', cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

aruco_marker_side_length = 0.08

# Inicializamos un diccionario para guardar las distancias, el origen de coordenadas para indicar que no existe actualmente y los diccionariois de ArUco
path = '.'
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
params = cv2.aruco.DetectorParameters_create()
Poses = {}
origin = -1
while True:
    # Detectamos los ArUcos
    frame = vs.read()
    frame = imutils.resize(frame, width=1080)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, dict, parameters=params, cameraMatrix=mtx, distCoeff=dst)

    if len(corners) > 0:
        # Si no tenemos un origen de coordenadas, tomamos el primer marcador como tal
        if origin == -1:
            origin = ids[0][0]
            print("Origen establecido en {}".format(origin))

        # Tomamos los vectores de rotación y traslación
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_marker_side_length, mtx, dst)
        # tvecs: Vector de translación en 3 dimensiones
        # rvecs: Vector de rotación

        for i, marker_id in enumerate(ids):
            if not marker_id[0] == origin:
                if not marker_id[0] in Poses:   
                    j = 0
                    Found = False
                    while j < len(ids) - 1 and not Found:
                        if ids[j][0] == origin: #or ids[j][0] in Poses:
                            Found = True
                        else:
                            j = j + 1
                        print("J: {}, Id: {}, Origin: {}".format(j, ids[j][0], origin))
                        
                    
                    # ¿Hemos encontrado algo? ¿Qué?
                    if ids[j][0] == origin:                        
                        Poses[ids[i][0]] = utils.calcPoseDirectly(tvecs, rvecs, i, j, marker_id, Debug=True)
                        print("Direct")
                        print(ids[i][0])
                        print(Poses[ids[i][0]])
                        break
                    elif ids[j][0] in Poses:
                        #Hemos encontrado otro marcador con distancia al origen ya definida, calculamos la pose del ArUco encadenando transformaciones
                        Poses[ids[i][0]] = utils.calcPoseIndirectly(tvecs, rvecs, i, j, Poses, ids, Debug=True)
                        print("Indirect")
                        print(ids[i][0])
                        print(Poses[ids[i][0]])
                        break
                    else:
                        #No hemos encontrado nada, mensaje de error
                        break
            else:
                # Para poder hacer los cálculos de la pose directos cuando está el origen en la imagen, guardaremos la pose de la cámara en el diccionario de las poses relativas
                # al origen. Siempre lo tendremos disponible, ya que el primero 
                # 
                # Bueno claro crack pero en el siguiente frame a lo mejor la cámara se ha movido pero el origen sigue en pantalla 
                # Poses[marker_id] = utils.calcPoseToTheCameraDirectly(tvecs, rvecs, i, marker_id)
                break


        
    
# Display the resulting frame
    cv2.imshow("Camara", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop() 

