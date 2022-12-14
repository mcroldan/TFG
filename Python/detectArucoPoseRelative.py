import cv2
import time
from imutils.video import VideoStream
import imutils
import json
import numpy as np
import zmq
import argparse
import utils.utils as utils

parser = argparse.ArgumentParser(description='Parameters for movement detection system')
parser.add_argument('-s', '--size', dest='size', required=True, type=float, help='Side length of ArUco Markers  (in meters)')
parser.add_argument('-c', '--calibration', dest='calibration', required=True, type=str, help='File path with previously calculated calibration parameters (with camera_calibration.py)')

args = parser.parse_args()
calibrationFile = args.calibration if args.calibration else './calibration_chessboard.yaml'

# We start the ZeroMQ socket to communicate with Unity. Python will be the client, which continously sends a 6x1 int array [coord_x, coord_y, coord_z, pitch_x, roll_y, yaw_z]
context = zmq.Context()

print("CREATING LOCALHOST SERVER...")
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5555")
print("LISTENING ON PORT 5555")

# Starting the Camera
vs = VideoStream(src=0).start()
time.sleep(2.0)

# Loading up the parameters obtained in the previous camera calibration
cv_file = cv2.FileStorage(
    calibrationFile, cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

aruco_marker_side_length = args.size # if args.size else 0.037 # 0.037
Debug_Markers = False
Debug_Camera = False

# We initialize a dictionary to save the poses, a placeholder world origin, and the ArUco dictionary 
path = '.'
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
params = cv2.aruco.DetectorParameters_create()
Poses = {}
origin = -1
frame_values = []

while True:
    frame = vs.read()
    frame = imutils.resize(frame, height=1920, width=1080)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, dict, parameters=params, cameraMatrix=mtx, distCoeff=dst)

    utils.drawMarkerFeatures(frame, rejected, (0,0,255))
    utils.drawMarkerFeatures(frame, corners, (0,255,0))

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
                if not marker_id[0] in Poses:   
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

                    elif ids[found_index][0] in Poses:
                        # We found another marker with its pose relative to the origin already calculated. We can concatenate transformations to get the pose we need
                        Poses[ids[marker_index][0]] = utils.calcPoseIndirectly(tvecs, rvecs, marker_index, found_index, Poses, ids, Debug=Debug_Markers)

                
                # We aren't currently on the origin, but we have the pose of the current marker to the origin already calculated. That said, we can compute the pose of the
                # camera with that relative pose.
                try:
                    _, pose_dict_quat = utils.calcCameraPoseIndirectly(tvecs, rvecs, marker_index, Poses, marker_id, Debug=Debug_Camera)
                except TypeError:
                    print("")
            else:
                _, pose_dict_quat = utils.calcCameraPoseDirectly(tvecs, rvecs, marker_index, ids, Debug=Debug_Camera)
            
            if pose_dict_quat != 0:
                pose_dict_quat = utils.roundPose(pose_dict_quat)
                frame_values.append(list(pose_dict_quat.values()))

        if len(frame_values) >= 8:
            median_pose = np.median(np.array(frame_values), axis=0)
            median_pose_dict =  {'pos_x': median_pose[0],
                                'pos_y': median_pose[1],
                                'pos_z': median_pose[2],
                                'quat_x': median_pose[3],
                                'quat_y': median_pose[4],
                                'quat_z': median_pose[5],
                                'quat_w': median_pose[6]}
            print(marker_id, origin, ":", median_pose_dict)
            json_str = json.dumps(median_pose_dict, ensure_ascii=False)
            socket.send_string(json_str)
            frame_values = []
        

# Display the resulting frame
    cv2.imshow("Camara", imutils.resize(frame, height=800, width=600))
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()