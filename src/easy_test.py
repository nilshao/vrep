# Author: Mingchuan ZHOU
# Contact: mingchuan.zhou@in.tum.de
# Date: 31 Jan. 2021
# Usage: for testing the easy image from CoppeliaSim via python remoteApi
import sys
sys.path.append('python')
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import cv2.aruco as aruco

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICTIONARY = aruco.Dictionary_get(aruco.DICT_4X4_1000)
ARUCO_SIZE_METER = 0.10

print('Program started')
try:
    import sim as vrep
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
import time
print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections



def get_ori_pos(obj,rel):
    res, rel_handle = vrep.simxGetObjectHandle(clientID, rel, vrep.simx_opmode_streaming)
    res, obj_handle = vrep.simxGetObjectHandle(clientID, obj, vrep.simx_opmode_streaming)
    res, pos = vrep.simxGetObjectPosition(clientID, obj_handle, rel_handle, vrep.simx_opmode_streaming)
    res, ori = vrep.simxGetObjectOrientation(clientID, obj_handle, rel_handle, vrep.simx_opmode_streaming)
    res, quat = vrep.simxGetObjectQuaternion(clientID, obj_handle, rel_handle, vrep.simx_opmode_streaming)
    return pos,quat

def get_trans_matrix(pos,ori):
    transformation_matrix = np.array([[0, 0, 0, 0],
                                      [0, 0, 0, 0],
                                      [0, 0, 0, 0],
                                      [0, 0, 0, 1]],
                                     dtype=float)
    try:
        r = R.from_quat(ori)
        transformation_matrix[0][3] = pos[0]
        transformation_matrix[1][3] = pos[1]
        transformation_matrix[2][3] = pos[2]

        transformation_matrix[:3, :3] = r.as_matrix()
    except:
        print('failed to compute transformation matrix')
    return transformation_matrix


clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if (clientID != -1) :
    print('Connected to remote API server')
    res, v0 = vrep.simxGetObjectHandle(clientID, 'Vision_global_rgb', vrep.simx_opmode_oneshot_wait)
    res, v1 = vrep.simxGetObjectHandle(clientID, 'Vision_global_depth', vrep.simx_opmode_oneshot_wait)
    res, v2 = vrep.simxGetObjectHandle(clientID, 'Vision_calibration', vrep.simx_opmode_oneshot_wait)

    res, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_streaming)
    res, resolution, image2 = vrep.simxGetVisionSensorImage(clientID, v2, 0, vrep.simx_opmode_streaming)

    imcount = 0
    while (vrep.simxGetConnectionId(clientID) != -1):
        print(imcount)

        res, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_buffer)
        res, resolution, image2 = vrep.simxGetVisionSensorImage(clientID, v2, 0, vrep.simx_opmode_streaming)

        if res == vrep.simx_return_ok:
            imcount = imcount + 1
            res, rgb_resolution, rgb_image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_buffer)
            res, depth_resolution, depth_image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_oneshot_wait)

            rgb_img = np.array(rgb_image, dtype=np.uint8)
            depth_img = np.array(depth_image, dtype=np.uint8)
            index = []
            for i in range(depth_resolution[0] * depth_resolution[1] * 3):
                if (i % 3 != 0):
                    index.append(i)
            depth_img = np.delete(depth_img, index)
            rgb_img.resize([rgb_resolution[1], rgb_resolution[0], 3])
            depth_img.resize([depth_resolution[1], depth_resolution[0], 1])
            rgb_img = cv2.flip(rgb_img, 0)
            # depth_img = cv2.flip(depth_image, 0)
####################Detect Marker#########################
            print('-------------Marker Part Start-------------')
            res, rgb_resolution, rgb_image2 = vrep.simxGetVisionSensorImage(clientID, v2, 0, vrep.simx_opmode_buffer)
            rgb_img2 = np.array(rgb_image2, dtype=np.uint8)
            rgb_img2.resize([rgb_resolution[1], rgb_resolution[0], 3])
            rgb_img2 = cv2.flip(rgb_img2, 0)

            res, v2 = vrep.simxGetObjectHandle(clientID, 'Vision_calibration', vrep.simx_opmode_oneshot_wait)

            cv2.imshow("RGB_Image", rgb_img2)
        #    cv2.imshow("DEPTH_Image", depth_img)
            pic_ori = rgb_img2
            pic_gray = cv2.cvtColor(pic_ori, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Gray_Image", pic_gray)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)  # Use 5x5 dictionary to find markers
            parameters = aruco.DetectorParameters_create()  # Marker detection parameters
            corners, ids, rejected_img_points = aruco.detectMarkers(pic_gray, ARUCO_DICTIONARY, parameters=ARUCO_PARAMETERS)
            print(ids)

            if np.all(ids is not None):  # If there are markers found by detector
                num_of_markers = ids.size
                res = aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE_METER, (matrix_coefficients),
                                                      (distortion_coefficients))
                rvec = res[0]
                tvec = res[1]
                #  markerPoints=res[2]

                aruco.drawDetectedMarkers(pic_gray, corners)  # Draw A square around the markers


            print('-------------Marker Part End---------------')
##########################################################
            marker00_to_joint3_pos,marker00_to_joint3_ori = get_ori_pos('Marker_4x4_1000_0', 'Franka_joint3')
            trans_matrix_marker00_to_joint3 = get_trans_matrix(marker00_to_joint3_pos,marker00_to_joint3_ori)

      #      print(trans_matrix_marker00_to_joint3)

            pos2, ori2 = get_ori_pos('Franka_joint3','Marker_4x4_1000_0')
            trans_2 = get_trans_matrix(pos2, ori2)
         #   print(trans_2)
            print(np.multiply(trans_2,trans_matrix_marker00_to_joint3))


            cv2.waitKey(1)
            # time.sleep(1)

        else:
            print('Failed to show rgb and depth image')
else:
    print('Failed connecting to remote API server')
print('Program ended')