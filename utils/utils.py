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
def calcPoseDirectly(tvecs, rvecs, marker_index, found_index, marker_id, Debug=False):
    '''If we have a marker and the origin on the screen, we can compute the pose of the marker relative to the origin directly
    tvecs = Translation vectors detected relative to the camera\n
    rvecs = Rotation vectors detected relative to the camera\n
    marker_index = Index of the marker that we will use to compute its pose relative to the origin\n
    found_index = Index of the origin\n
    marker_id = Identifier of the marker that we will use to compute its pose relative to the origin\n
    debug = Enable / Disable debug mode. If enabled, you will get more information as logs in the console\n'''

    # We found the world's origin in the image, so we can calculate the AruCo's pose relative to the origin directly.
    pose_marker_to_camera = np.eye(4)
    pose_marker_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[marker_index][0]))[0]
    pose_marker_to_camera[0:3, 3] = tvecs[marker_index][0]

    pose_marker_to_camera = correctInvertedPose(pose_marker_to_camera)

    pose_origin_to_camera = np.eye(4)
    pose_origin_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[found_index][0]))[0]
    pose_origin_to_camera[0:3, 3] = tvecs[found_index][0] # translation_camera_to_origin

    print("\n\n\nOriginal:")
    print(pose_origin_to_camera)
    pose_origin_to_camera = correctInvertedPose(pose_origin_to_camera)
    print("Corregida:")
    print(pose_origin_to_camera, "\n\n\n\n")

    pose_camera_to_origin = np.linalg.inv(pose_origin_to_camera)

    pose = pose_camera_to_origin @ pose_marker_to_camera
    if Debug:
        t_matrix_to_angles(pose, marker_id, True)
        print("")
        #t_matrix_to_angles(np.linalg.inv(pose_marker_to_camera), marker_id, True)
    return pose

def calcPoseIndirectly(tvecs, rvecs, marker_index, found_index, Poses, marker_ids, Debug=False):
    '''If we have two markers, one of them with its pose relative to the origin already calculated, we can get the first marker's pose relative to the origin indirectly.\n 
    tvecs = Translation vectors detected relative to the camera\n
    rvecs = Rotation vectors detected relative to the camera\n
    Poses = Diccionario con las poses calculadas entre un marcador y el origen
    marker_index = Index of the marker that we will use to compute its pose relative to the origin\n
    found_index = Index of the marker with the pose relative to the origin already calculated\n
    marker_id = Identifier of the marker that we will use to compute its pose relative to the origin\n
    debug = Enable / Disable debug mode. If enabled, you will get more information as logs in the console\n'''
    
    # We didn't find the world's origin in the image. However, we got a marker with its pose relative to the origin already computed.
    pose_marker_to_known = calcPoseDirectly(tvecs, rvecs, marker_index, found_index, marker_ids[marker_index][0])

    pose_known_to_origin = Poses[marker_ids[found_index][0]]

   # pose = pose_marker_to_camera @ pose_camera_to_known @ pose_known_to_origin
    pose = pose_known_to_origin @ pose_marker_to_known
    if Debug:
        t_matrix_to_angles(pose, marker_ids[marker_index][0], True)
    return pose

def calcCameraPoseIndirectly(tvecs, rvecs, marker_index, Poses, marker_id, Debug=False):
    # We found the world's origin in the image, so we can calculate the AruCo's pose relative to the origin directly.
    pose_known_to_camera = np.eye(4)
    pose_known_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[marker_index][0]))[0]
    pose_known_to_camera[0:3, 3] = tvecs[marker_index][0]

    pose_known_to_camera = correctInvertedPose(pose_known_to_camera)

    if not marker_id[0] in Poses:
        return

    pose_known_to_origin = Poses[marker_id[0]]

    pose_camera_to_known = np.linalg.inv(pose_known_to_camera)

    pose = pose_known_to_origin @ pose_camera_to_known
    if Debug:
        t_matrix_to_angles(pose, marker_id[0], True)
        print("")
        #t_matrix_to_angles(np.linalg.inv(pose_marker_to_camera), marker_id, True)
    return pose

def calcCameraPoseDirectly(tvecs, rvecs, marker_index, marker_ids, Debug=False):
    # We found the world's origin in the image, so we can calculate the AruCo's pose relative to the origin directly.
    pose_origin_to_camera = np.eye(4)
    pose_origin_to_camera[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[marker_index][0]))[0]
    pose_origin_to_camera[0:3, 3] = tvecs[marker_index][0]

    pose_origin_to_camera = correctInvertedPose(pose_origin_to_camera)

    pose = np.linalg.inv(pose_origin_to_camera)
    if Debug:
        t_matrix_to_angles(pose, marker_ids[marker_index][0], True)
        print("")
        #t_matrix_to_angles(np.linalg.inv(pose_marker_to_camera), marker_id, True)
    return pose
    
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

# (0,0,255) = red
# (255,0,0) = green
def drawMarkerFeatures(img, corners, color):
    ids = [1, 2, 3]
    for(marker, id) in zip(corners, ids):
        corners = marker.reshape(4, 2)
        (topLeft, topRight, botRight, botLeft) = corners
        
        # We mark the corners and the center in red
        cv2.circle(img, topLeft.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, topRight.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, botRight.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, botLeft.astype(int), 1, (0,0,255), -1)
        cv2.circle(img, (int((topLeft[0] + botRight[0])/2), int((topLeft[1] + botRight[1])/2)), 1, (0,0,255), -1)

        # We mark the outline in green
        cv2.line(img, topLeft.astype(int), topRight.astype(int), color, 2)
        cv2.line(img, topLeft.astype(int), botLeft.astype(int), color, 2)
        cv2.line(img, botLeft.astype(int), botRight.astype(int), color, 2)
        cv2.line(img, botRight.astype(int), topRight.astype(int), color, 2)

def cross(a:np.ndarray,b:np.ndarray)->np.ndarray:
    return np.cross(a, b)

def correctInvertedPose(Pose):
    T = Pose[0:3, 3]
    R = Pose[0:3, 0:3]

    if 0 < R[1,1] < 1:
        # If it gets here, the pose is flipped.

        # Flip the axes. E.g., Y axis becomes [-y0, -y1, y2].
        R *= np.array([
            [ 1, -1,  1],
            [ 1, -1,  1],
            [-1,  1, -1],
        ])
        
        # Fixup: rotate along the plane spanned by camera's forward (Z) axis and vector to marker's position
        forward = np.array([0, 0, 1])
        tnorm = T / np.linalg.norm(T)
        axis = cross(tnorm, forward)
        print("AGUA")
        angle = -2*math.acos(tnorm @ forward)
        R = cv2.Rodrigues(angle * axis)[0] @ R
    
    Pose[0:3, 3] = T
    Pose[0:3, 0:3] = R
    return Pose