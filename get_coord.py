import rospy
import matplotlib.pylab as plt
import time
import pickle
import cv2
import numpy as np
import json
from roboflow import Roboflow

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib.pyplot import figure

class GetCoord(object):
    def __init__(self):
        #rospy.init_node("take_pic")

        self.bridge = CvBridge()
        self.plt = plt
        self.time = time

    def process(self):
        img_rgb = rospy.wait_for_message("/camera/color/image_raw",Image)
        cv_image_rgb = self.bridge.imgmsg_to_cv2(img_rgb, desired_encoding='passthrough')
        rf = Roboflow(api_key="XqbFaKHYp31Oe9UITvyq")
        project = rf.workspace().project("robothon-1eusu")
        model = project.version(4).model
        obj = model.predict(cv_image_rgb, confidence=50, overlap=30).json()
        my_filter = filter(blue_button, obj["predictions"])
        filtered_obj = list(my_filter)
        coord_x = filtered_obj[0]['x'] + filtered_obj[0]['width']/2
        coord_y = filtered_obj[0]['y'] + filtered_obj[0]['height']/2
        l = [coord_x, coord_y]
        return l

def blue_button(elem):
    return elem["class"] == "blue-button"
