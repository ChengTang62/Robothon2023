import rospy
import matplotlib.pylab as plt
import pickle
import time
import pickle
import cv2
import numpy as np


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib.pyplot import figure

class TakePicture(object):
    def __init__(self):
        #rospy.init_node("take_pic")

        self.bridge = CvBridge()
        self.plt = plt
        self.time = time
        

    def take_pic_rgb(self):
        img_rgb = rospy.wait_for_message("/camera/color/image_raw",Image)
        cv_image_rgb = self.bridge.imgmsg_to_cv2(img_rgb, desired_encoding='passthrough')
        return cv_image_rgb

    def take_pic_depth(self):
        self.img_depth = rospy.wait_for_message("/camera/depth/image_rect_raw",Image)
        self.cv_image_depth = self.bridge.imgmsg_to_cv2(self.img_depth, desired_encoding='passthrough')
        return self.cv_image_depth

    def save_pic_rgb_timed(self,sleep_time,num_img):
        for i in range(num_img):
            filename = "data/board11/img_" + str(i) + ".pickle"
            img = self.take_pic_rgb()
            self.pickle_to(img,filename)
            time.sleep(sleep_time)

    def save_pic_jpg_timed(self,sleep_time,num_img,dir):
        for i in range(num_img):
            img = self.take_pic_rgb()
            des = dir + 'img' + str(i) + '.jpg'
            img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
            cv2.imwrite(des,img)
            time.sleep(sleep_time)


    def show_pic(self,img):
        figure(figsize=(20,20),dpi=80)
        self.plt.imshow(img)

    def pickle_to(self,img,dir):
        pickle.dump(img,open(dir,"wb"))

    def get_pickle(self,dir):
        loaded_img = pickle.load(open(dir,"rb"))
        return loaded_img

    def get_green(self):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        
        params.filterByColor = False

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255
        params.thresholdStep = 1
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100
        
        # Filter by Circularityrc.reset()
        params.filterByCircularity = False
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
        
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        raw_img_bgr = self.take_pic_rgb()
        crop_img_bgr = raw_img_bgr[660:710,830:1050]

        hsv = cv2.cvtColor(crop_img_bgr, cv2.COLOR_BGR2HSV) #HSV



        #img = tp.take_pic_rgb()
        #im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        # Threshold of blue in HSV space

        # HSV for cv2: [0-179,0-255,0-255]

        #lower_blue = np.array([30, 0, 0])
        #upper_blue = np.array([100, 255, 255])
        #mask_no_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        #img_no_blue = cv2.bitwise_and(crop_img_bgr, crop_img_bgr, mask = mask_no_blue)

        lower_green = np.array([31, 100, 0])
        upper_green = np.array([120, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        img_green = cv2.bitwise_and(crop_img_bgr, crop_img_bgr, mask = mask_green)

        
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create(params)
        
        # Detect blobs.
        keypoints = detector.detect(img_green)
        
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        img_with_keypoints = cv2.drawKeypoints(img_green, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


            
        plt.imshow(img_with_keypoints)

        if len(keypoints) == 1:
            return keypoints[0].pt[0]
        elif len(keypoints) == 0:
            return 0.0
        else:
            return False

        
    
    def get_yellow(self):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        
        params.filterByColor = False

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255
        params.thresholdStep = 1
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100
        
        # Filter by Circularityrc.reset()
        params.filterByCircularity = False
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
        
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        raw_img_bgr = self.take_pic_rgb()
        crop_img_bgr = raw_img_bgr[660:710,830:1100]

        hsv = cv2.cvtColor(crop_img_bgr, cv2.COLOR_BGR2HSV) #HSV



        #img = tp.take_pic_rgb()
        #im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        # Threshold of blue in HSV space

        # HSV for cv2: [0-179,0-255,0-255]

        #lower_blue = np.array([30, 0, 0])
        #upper_blue = np.array([100, 255, 255])
        #mask_no_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        #img_no_blue = cv2.bitwise_and(crop_img_bgr, crop_img_bgr, mask = mask_no_blue)

        lower_green = np.array([31, 0, 0])
        upper_green = np.array([120, 100, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        img_green = cv2.bitwise_and(crop_img_bgr, crop_img_bgr, mask = mask_green)

        
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create(params)
        
        # Detect blobs.
        keypoints = detector.detect(img_green)
        
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        img_with_keypoints = cv2.drawKeypoints(img_green, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


            
        plt.imshow(img_with_keypoints)

        if len(keypoints) == 1:
            return keypoints[0].pt[0]
        elif len(keypoints) == 0:
            return 0.0
        else:
            return False

    def get_slider_dist(self):


        pixel = self.get_yellow()-self.get_green()
        #dist = 1.9*pixel/10000


        if pixel < 0:
            dist = 1.5*pixel/10000
            print("other side: " + str(dist))

        else:
            dist = 1.5*pixel/10000  
            print("same side: " + str(dist))
  

        return(dist)