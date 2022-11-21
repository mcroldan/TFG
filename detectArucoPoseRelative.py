from multiprocessing.connection import wait
import cv2
import time
from imutils.video import VideoStream
import imutils
import time
import json

import utils.utils as utils

# We start the ZeroMQ socket to communicate with Unity. Python will be the client, which continously sends a 6x1 int array [coord_x, coord_y, coord_z, pitch_x, roll_y, yaw_z]
"""context = zmq.Context()

print("CONNECTING TO LOCALHOST SERVER...")
socket = context.socket(zmq.REP)
socket.connect("tcp://localhost:5555")
print("CONNECTED!")"""

# Starting the Camera
vs = VideoStream(src=0).start()
time.sleep(2.0)

# Loading up the parameters obtained in the previous camera calibration
cv_file = cv2.FileStorage(
    'calibration_chessboard_camera.yaml', cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

aruco_marker_side_length = 0.037 #0.08
Debug_Markers = False
Debug_Camera = False

# We initialize a dictionary to save the poses, a placeholder world origin, and the ArUco dictionary 
path = '.'
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
params = cv2.aruco.DetectorParameters_create()
Poses = {}
origin = -1
toy = 1
while True:
    # Loading the image / camera frame
    frame = vs.read()
    #path = './examples/'
    
    #frame = cv2.imread(path + 'toy2-camera.PNG')

    frame = imutils.resize(frame, height=1920, width=1080)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, dict, parameters=params, cameraMatrix=mtx, distCoeff=dst)

    utils.drawMarkerFeatures(frame, rejected, (0,0,255))
    utils.drawMarkerFeatures(frame, corners, (0,255,0))

    #frame = outliers.applyHarrisToUndistortedImg(outliers.solveDistortedImage(frame, corners))


    if len(corners) > 0:
        # If we don't have a world origin, we set it to be the first market we can find
        if origin == -1:
            origin = ids[0][0]
            print("Origen establecido en {}".format(origin))

        # Rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_marker_side_length, mtx, dst)
        # tvecs: Translation vector in 3D
        # rvecs: Rotation vector

        for marker_index, marker_id in enumerate(ids):
            if not marker_id[0] == origin:
                #print("Looking into", marker_id[0])
                if True: #not marker_id[0] in Poses:   
                    found_index = 0
                    Found = False
                    while found_index < len(ids) - 1 and not Found:
                        print("J: {}, Id: {}, Origin: {}".format(found_index, ids[found_index][0], origin))
                        if ids[found_index][0] == origin or ids[found_index][0] in Poses:
                            Found = True
                        else:
                            found_index = found_index + 1

                    # Did we find something? What?
                    if ids[found_index][0] == origin:
                        # We found the world origin, so we can calculate the pose from the marker directly
                        Poses[ids[marker_index][0]] = utils.calcPoseDirectly(tvecs, rvecs, marker_index, found_index, marker_id, Debug=Debug_Markers)
                        print("Direct")
                        #print(ids[marker_index][0])
                        #print(Poses[ids[marker_index][0]])
                        #break
                    elif ids[found_index][0] in Poses:
                        # We found another marker with its pose relative to the origin already calculated. We can concatenate transformations to get the pose we need
                        Poses[ids[marker_index][0]] = utils.calcPoseIndirectly(tvecs, rvecs, marker_index, found_index, Poses, ids, Debug=Debug_Markers)
                        print("Indirect")
                        #print(ids[marker_index][0])
                        #print(Poses[ids[marker_index][0]])
                        #break
                    #else:
                        # We didn't find anything, so we can't do any operation
                    #    break
                
                # We aren't currently on the origin, but we have the pose of the current marker to the origin already calculated. That said, we can compute the pose of the
                # camera with that relative pose.
                #print("Indirect, Origin:{}".format(origin))
                _, pose_dict = utils.calcCameraPoseIndirectly(tvecs, rvecs, marker_index, Poses, marker_id, Debug=Debug_Camera)
                #print(ids[marker_index][0])
                #print(Poses[ids[marker_index][0]])
            else:
                #print("Direct, Origin:{}".format(origin))
                _, pose_dict = utils.calcCameraPoseDirectly(tvecs, rvecs, marker_index, ids, Debug=Debug_Camera)
                #print(ids[marker_index][0])
            
            if pose_dict != 0:
                print(pose_dict)

    time.sleep(0.1)
# Display the resulting frame
    cv2.imshow("Camara", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop() 

