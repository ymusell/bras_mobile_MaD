#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_one_python_api.niryo_one_api import *
import math
import rospy
import time
import numpy as np
import cv2

import markers_detection as m
import math_functions as math


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
    # n.move_joints([-0.043, 0.245, -0.1, 0.066, -1.639, -2.566])

    n.wait(1)
    img = n.get_compressed_image()
    img = f(img)

    filename = 'savedImage2.jpg'
    cv2.imwrite(filename, img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    img_thresh = cv2.adaptiveThreshold(gray, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
                          thresholdType=cv2.THRESH_BINARY, blockSize=15, C=25)

    list_good_candidates = m.find_markers_from_img_thresh(img_thresh)
    print(len(list_good_candidates))
    for k in range (4):
        print(list_good_candidates[k].cx , list_good_candidates[k].cy)
    n.activate_learning_mode(True)

except NiryoOneException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"
