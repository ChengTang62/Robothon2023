import rospy
import matplotlib.pylab as plt
import time
import pickle
import cv2
import numpy as np
import json
from roboflow import Roboflow
import math
import functions as rc
import take_pic

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib.pyplot import figure

class GetOrien(object):
    def __init__(self):
        #rospy.init_node("take_pic")

        self.bridge = CvBridge()
        self.plt = plt
        self.time = time
        self.tp = take_pic.TakePicture()

    def process(self):
        img_rgb = self.tp.take_pic_rgb()
        img_rgb = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2RGB)
        des = "data/board/board_calib0/calib2.jpg"
        cv2.imwrite(des,img_rgb)
        rf = Roboflow(api_key="XqbFaKHYp31Oe9UITvyq")
        project = rf.workspace().project("robothon-1eusu")
        model = project.version(4).model
        obj = model.predict(img_rgb, confidence=40, overlap=30).json()
        bb_filter = filter(blue_button, obj["predictions"])
        rtp_filter = filter(red_test_point, obj["predictions"])
        filtered_bb = list(bb_filter)
        filtered_rtp = list(rtp_filter)
        coord_x_bb = filtered_bb[0]['x'] + filtered_bb[0]['width']/2
        coord_y_bb = filtered_bb[0]['y'] + filtered_bb[0]['height']/2
        coord_x_rtp = filtered_rtp[0]['x'] + filtered_rtp[0]['width']/2
        coord_y_rtp = filtered_rtp[0]['y'] + filtered_rtp[0]['height']/2
        diff_x = coord_x_bb - coord_x_rtp
        diff_y = coord_y_rtp - coord_y_bb
        tangent = diff_y/diff_x
        angle = math.atan(tangent)
        angle_to_turn = convert(angle)
        return angle_to_turn
    
    def get_angle_diff(self):
        converted_angle = self.process()
        return converted_angle-math.pi/4

def blue_button(elem):
    return elem["class"] == "blue-button"
def red_test_point(elem):
    return elem["class"] == "red-test-point"

def convert(angle):
    return  math.pi/4 + 0.348 - angle



def turn(angle):
    rc.pose_to(xyzw)