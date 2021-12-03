#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_one_python_api.niryo_one_api import *
import math
import rospy
import time
import numpy as np
import cv2
import sys

import markers_detection as m

#workspace_ratio = n.get_workspace_ratio('default_workspace')
workspace_ratio = 1.0
check_workspace = 1


def f(compressed_image):
    np_arr = np.fromstring(compressed_image, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img


rospy.init_node('niryo_one_example_python_api')
print "--- Start"

n = NiryoOne()

try:
    # Calibrate robot first
    n.calibrate_auto()
    print "Calibration finished !"

    n.activate_learning_mode(False)

    print "Go to observation position"
    n.move_joints([-0.027, 0.373, -0.203, 0.04, -1.576, -2.566])

    n.wait(1)
    img = n.get_compressed_image()
    img = f(img)

    filename = '/home/niryo/catkin_ws/src/niryo_one_python_api/examples/imageMarkTest.jpg'
    cv2.imwrite(filename, img)

    
    #################################################################################################
    
    # Reading an image in default mode
    img = cv2.imread(filename)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    img_thresh = cv2.adaptiveThreshold(gray, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
                            thresholdType=cv2.THRESH_BINARY, blockSize=15, C=25)

    list_good_candidates = m.find_markers_from_img_thresh(img_thresh)


    if not list_good_candidates or len(list_good_candidates) > 6:
        check_workspace = 0

    if len(list_good_candidates) == 4:
        list_good_candidates = m.sort_markers_detection(list_good_candidates)
    else:
        list_good_candidates = m.complicated_sort_markers(list_good_candidates, workspace_ratio=workspace_ratio)
        if list_good_candidates is None:
            check_workspace = 0

    # print(list_good_candidates)
    # print(check_workspace)
    # print(list_good_candidates[0].cx, list_good_candidates[0].cy)

    temoin = [[218,52],[411,56],[415,252],[200,245]]

    test = []

    for i in range (4) :
        test.append([list_good_candidates[i].cx , list_good_candidates[i].cy]) #liste des centres des marqueurs      

    # for i in range (4) :
    #     print(test[i])

    lx = 0
    ly = 0

    for i in range (4) :
        lx += temoin[i][0]-test[i][0]
        ly += temoin[i][1]-test[i][1]

    lx = lx/4.0
    ly = ly/4.0


    print("-----------------------------")
    print("translation en px",lx,ly)

    mm = 172.0/215.0/1.5 #Il y a 172mm entre le centre de deux marqueurs

    tx = mm*lx
    ty = mm*ly

    print("-----------------------------")
    print("translation en mm",tx,ty)


    v_ws = [test[2][0]-test[3][0], test[2][1]-test[3][1]] #vecteur des deux marqueurs gauche du workspace
    u_ws = [temoin[2][0]-temoin[3][0], temoin[2][1]-temoin[3][1]]

    # angle = atan2(vector2.y, vector2.x) - atan2(vector1.y, vector1.x) --> angle entre le cote du bas du workspace et l'arrete basse de la camera
    angle = np.arctan2(v_ws[1],v_ws[0]) - np.arctan2(0,100)  
    # angle = math.acos((np.dot(v_ws,u_ws))/(np.linalg.norm(u_ws)*np.linalg.norm(v_ws)))

    if angle > np.pi  :
        angle -= 2*np.pi

    if angle <= -np.pi :
        angle += 2*np.pi
      
    print("-----------------------------")
    print(angle)

    success, new_img = m.draw_markers(img, workspace_ratio=1.0)
    # img2 = cv2.imread('/home/niryo/catkin_ws/src/niryo_one_python_api/examples/imageMarkTest.jpg')
    # success, new_img_2 = m.draw_markers(img2, workspace_ratio=1.0)
    # cv2.imshow('image',new_img_2)
    cv2.imshow('image',new_img)

    # n.move_joints([0.019, 0.101, -1.08, 0.06, -0.573, -2.556])
    n.activate_learning_mode(True)


except NiryoOneException as e:
    print e

print "--- End"
# os.system('rm /home/niryo/catkin_ws/src/niryo_one_python_api/examples/imageMarkTest.jpg')
