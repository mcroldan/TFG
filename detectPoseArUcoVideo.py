from tkinter import Frame
import numpy as np
import cv2
import time
from imutils.video import VideoStream
import imutils
import math
from scipy.spatial.transform import Rotation as R

def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
      
  return roll_x, pitch_y, yaw_z # in radians

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

while True:
    path = './examples/'
    #path = '.'
    dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    params = cv2.aruco.DetectorParameters_create()
    frame = cv2.imread(path + 'toy5.PNG', cv2.IMREAD_GRAYSCALE)
    #frame = vs.read()
    #frame = imutils.resize(frame, width=1080)
    dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    params = cv2.aruco.DetectorParameters_create()

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, dict, parameters=params, cameraMatrix=mtx, distCoeff=dst)

    if len(corners) > 0:
        # Tomamos los vectores de rotación y traslación
        # detectArUcoVideo.drawMarkerFeatures(frame, corners, ids)
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_marker_side_length, mtx, dst)
        # tvecs: Vector de translación en 3 dimensiones
        # rvecs: Vector de rotación

        for i, marker_id in enumerate(ids):
        
            # Store the translation (i.e. position) information
            transform_translation_x = tvecs[i][0][0]
            transform_translation_y = tvecs[i][0][1]
            transform_translation_z = tvecs[i][0][2]

            # Store the rotation information
            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            quat = r.as_quat()   
            
            """# Quaternion format     
            transform_rotation_x = quat[0] 
            transform_rotation_y = quat[1] 
            transform_rotation_z = quat[2] 
            transform_rotation_w = quat[3] 
            
            # Euler angle format in radians
            roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                        transform_rotation_y, 
                                                        transform_rotation_z, 
                                                        transform_rotation_w)
            
            roll_x = math.degrees(roll_x)
            pitch_y = math.degrees(pitch_y)
            yaw_z = math.degrees(yaw_z)
            print("transform_translation_x: {}".format(transform_translation_x))
            print("transform_translation_y: {}".format(transform_translation_y))
            print("transform_translation_z: {}".format(transform_translation_z))
            print("roll_x: {}".format(roll_x))
            print("pitch_y: {}".format(pitch_y))
            print("yaw_z: {}".format(yaw_z))
            print()"""
            
            # Draw the axes on the marker
            cv2.aruco.drawAxis(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)

            # En rotation_matrix tenemos la matriz de rotación de la pose del ArUco Marker. Sin embargo, si le hacemos la inversa podemos obtener la de la cámara.
            print('-----------------------------------------------------')
            print('Marcador: {}'.format(marker_id))
            camera_rotation_matrix = np.transpose(rotation_matrix)


            # Para la traslación de la cámara, simplemente necesitamos volver negativa la del marcador. La explicación es trivial
            camera_translation_vector = []
            camera_translation_vector = [-1 * transform_translation_x, -1 * transform_translation_y, -1 * transform_translation_z]

            T = np.copy(camera_rotation_matrix)
            T[0:3,3] = camera_translation_vector

            print("Matriz de rotación de la pose de la cámara:\n {}".format(camera_rotation_matrix))
            print("Vector de traslación de la pose de la cámara: {}".format(camera_translation_vector))
            print("Matriz T:\n {}\n".format(T[0:3,0:4]))

            #h_frame = np.append(frame, np.ones((1,frame.shape[1])), axis=0)
        
    
# Display the resulting frame
    cv2.imshow("Camara", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop() 

