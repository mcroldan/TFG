from multiprocessing.connection import wait
from tkinter import Frame
import numpy as np
import cv2
import time
from imutils.video import VideoStream
import imutils
import math
from scipy.spatial.transform import Rotation as R

#asdf
def calcPoseDirectly(tvecs, rvecs, i, j, marker_id, Debug=False):
    '''Si disponemos de un marcador y el origen en pantalla, podemos hacer el cálculo de la pose entre ambos de forma directa.\n 
    tvecs = Vectores de traslación detectados\n
    rvecs = Vectores de rotación detectados\n
    i = Índice del marcador del que queremos detectar su posición respecto al origen\n
    j = Índice del origen\n
    marker_id = Identificador del marcador del que queremos detectar su posición respecto al origen\n
    debug = Activa o desactiva el modo debug para recibir logs en la consola con los valores calculados\n'''
    #Hemos encontrado el origen en la imagen, calculamos la pose del ArUco respecto al origen
    pose_marker_to_camera = np.eye(4)
    pose_marker_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
    pose_marker_to_camera[0:3, 3] = tvecs[i][0]

    rotation_origin_to_camera = np.eye(3)
    rotation_origin_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[j][0]))[0]

    # rotation_camera_to_origin = np.transpose(rotation_origin_to_camera)
    # Para la traslación de la cámara, simplemente necesitamos volver negativa la del marcador. La explicación es trivial
    translation_camera_to_origin = []
    translation_camera_to_origin = [-1 * tvecs[j][0][0], -1 * tvecs[j][0][1], -1 * tvecs[j][0][2]]

    pose_camera_to_origin = np.eye(4)
    pose_camera_to_origin[0:3, 0:3] = rotation_origin_to_camera
    pose_camera_to_origin[0:3, 3] = tvecs[j][0] # translation_camera_to_origin

    # pose = pose_marker_to_camera @ np.linalg.inv(pose_camera_to_origin)
    pose = np.linalg.inv(pose_camera_to_origin) @ pose_marker_to_camera
    if Debug:
        t_matrix_to_angles(pose, marker_id, True)

    """print("---------------------------")
    print("MARKER {} TO ORIGIN".format(ids[i]))
    print(pose)
    print("---------------------------")"""
    return pose

def calcPoseIndirectly(tvecs, rvecs, i, j, Poses, marker_ids, Debug=False):
    '''Si disponemos de un marcador y otro con la pose respecto al origen en pantalla, podemos hacer el cálculo de la pose entre ambos de forma indirecta.\n 
    tvecs = Vectores de traslación detectados\n
    rvecs = Vectores de rotación detectados\n
    Poses = Diccionario con las poses calculadas entre un marcador y el origen
    i = Índice del marcador del que queremos detectar su posición respecto al origen\n
    j = Índice del marcador con la distancia calculada\n
    marker_id = Identificador del marcador del que queremos detectar su posición respecto al origen\n
    debug = Activa o desactiva el modo debug para recibir logs en la consola con los valores calculados\n'''
    #No hemos encontrado el origen en la imagen, pero sí un marcador con la posición respecto al origen calculada

    """pose_marker_to_camera = np.eye(4)
    pose_marker_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
    pose_marker_to_camera[0:3, 3] = tvecs[i][0]

    rotation_known_to_camera = np.eye(3)
    rotation_known_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[j][0]))[0]

    rotation_camera_to_known = np.transpose(rotation_known_to_camera)
    # Para la traslación de la cámara, simplemente necesitamos volver negativa la del marcador. La explicación es trivial
    translation_camera_to_known = []
    translation_camera_to_known = [-1 * tvecs[j][0][0], -1 * tvecs[j][0][1], -1 * tvecs[j][0][2]]

    pose_camera_to_known = np.eye(4)
    pose_camera_to_known[0:3, 0:3] = rotation_camera_to_known
    pose_camera_to_known[0:3, 3] = translation_camera_to_known"""
    pose_marker_to_known = calcPoseDirectly(tvecs, rvecs, i, j, marker_ids[i][0])

    pose_known_to_origin = Poses[marker_ids[j][0]]

   # pose = pose_marker_to_camera @ pose_camera_to_known @ pose_known_to_origin
    pose = pose_known_to_origin @ pose_marker_to_known
    if Debug:
        t_matrix_to_angles(pose, marker_ids[i][0], True)
    """print("---------------------------")
    print("MARKER {} TO ORIGIN".format(ids[i]))
    print(pose)
    print("---------------------------")"""
    return pose

def calcPoseToTheCameraDirectly(tvecs, rvecs, i, marker_id):
    # Store the translation (i.e. position) information
        transform_translation_x = tvecs[i][0][0]
        transform_translation_y = tvecs[i][0][1]
        transform_translation_z = tvecs[i][0][2]

        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   

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

        return T
    
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

def t_matrix_to_angles(pose, marker_id, Debug=False):
    r = R.from_matrix(pose[0:3, 0:3])
    quat = r.as_quat()   
    
    # Quaternion format     
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

    if Debug:
        print("Marker Id: {}".format(marker_id))
        print("transform_translation_x: {}".format(pose[0,3]))
        print("transform_translation_y: {}".format(pose[1,3]))
        print("transform_translation_z: {}".format(pose[2,3]))
        print("roll_x: {}".format(roll_x))
        print("pitch_y: {}".format(pitch_y))
        print("yaw_z: {}".format(yaw_z))
        print()
