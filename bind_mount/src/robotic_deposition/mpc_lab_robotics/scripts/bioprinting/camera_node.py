#!/usr/bin/env python3
import cv2 
import numpy as np 
#from cv_utils import *
import os 
import glob
from numpy.linalg import norm  
import pandas as pd 
from vidstab import VidStab
import signal
import rospy
import time
import sys
from std_msgs.msg import Float64

class camera_node():
    def __init__(self, use_ros = True, frequency = 125):  
        self.use_ros = use_ros
        self.frequency = frequency

        #self.sim = True
        self.sim = False

        self.Width = 640
        self.Height = 480
        self.max_value = 255 

        if self.use_ros:
            self.initialize_ros()

        if self.sim == False:
            #connect camera
            self.vidcap = cv2.VideoCapture(0)
            self.stabilizer = VidStab()

            self.vidcap.set(cv2.CAP_PROP_AUTOFOCUS, 0) #turnsoff autofocus 
            self.vidcap.set(cv2.CAP_PROP_FOCUS, 130)
            self.vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, self.Width)
            self.vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.Height)
            self.vidcap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
            self.vidcap.set(cv2.CAP_PROP_CONTRAST, 128)
            self.vidcap.set(cv2.CAP_PROP_SATURATION, 255)
            self.vidcap.set(cv2.CAP_PROP_FPS, 30) 
            self.vidcap.set(cv2.CAP_PROP_EXPOSURE, 400)

        time.sleep(0.5)
        self.run_main_loop()

    def initialize_ros(self):
        signal.signal(signal.SIGINT, self.stop_camera) 
        rospy.init_node('line_width_talker', anonymous=True)
        self.rate =  rospy.Rate(self.frequency)
        self.line_width_publisher = rospy.Publisher('detected_line_width', Float64, queue_size=1)
        print("camera_node ROS initialized")
        

    def run_color_tracker(self):
        window_capture_name = 'Video Capture'
        window_detection_name = 'Object Detection'
        low_R_name = 'Low R'
        low_G_name = 'Low G'
        low_B_name = 'Low B'
        high_R_name = 'High R'
        high_G_name = 'High G'
        high_B_name = 'High B'

        color_filter_vals_min = [0, 160, 0 ]
        color_filter_vals_max = [100, 255, 240]
         
        cv2.namedWindow(window_detection_name)
        cv2.createTrackbar(low_R_name, window_detection_name , color_filter_vals_min[0], self.max_value, on_low_R_thresh_trackbar)
        cv2.createTrackbar(high_R_name, window_detection_name , color_filter_vals_max[0], self.max_value, on_high_R_thresh_trackbar)
        cv2.createTrackbar(low_G_name, window_detection_name , color_filter_vals_min[1], self.max_value, on_low_G_thresh_trackbar)
        cv2.createTrackbar(high_G_name, window_detection_name , color_filter_vals_max[1], self.max_value, on_high_G_thresh_trackbar)
        cv2.createTrackbar(low_B_name, window_detection_name , color_filter_vals_min[2], self.max_value, on_low_B_thresh_trackbar)
        cv2.createTrackbar(high_B_name, window_detection_name , color_filter_vals_max[2], self.max_value, on_high_B_thresh_trackbar)

        self.color_tracker = Tracker()

        # Everything up till this point is initializer
        count = 0
        success = True
        while True:
            grabbed_frame, frame = self.vidcap.read()

            # Pass frame to stabilizer even if frame is None
            stabilized_frame = self.stabilizer.stabilize_frame(input_frame=frame, border_size=50, smoothing_window=15)

            # If stabilized_frame is None then there are no frames left to process
            if stabilized_frame is None:
                break

            key = cv2.waitKey(33) #approx.30fps, 1000/x = yfps i.e.1000/10 = 100 fps< original:5
            
            if count>5: #1000: 
                xc, yc, mask, with_contours,ink_width= self.color_tracker.track(stabilized_frame, color_filter_vals_min, color_filter_vals_max) 

                m2 = np.dstack((mask,mask,mask)) 
                myObject=cv2.bitwise_and(stabilized_frame,stabilized_frame,mask=mask)
                images = np.hstack((with_contours, m2,myObject))
                font = cv2.FONT_HERSHEY_SIMPLEX 
                cv2.putText(images, f'Width = {ink_width}',(10,450), font, 0.8,(0,255,0),2,cv2.LINE_AA)

                cv2.imshow('image', images)
                #cv2.waitKey(0)
                #print(ink_width)

                self.line_width_publisher.publish(ink_width)

            count += 1

    def stop_camera(self, *args, **kwargs):
        print("Stopping camera")
        self.vidcap.release()
        time.sleep(0.5)
        # os._exit(0)
        sys.exit()
        # print("Exited")

    def run_main_loop(self):
        print("Starting camera loop")
        if self.use_ros: 
            while not rospy.is_shutdown():
                self.run_color_tracker()
        else:
            while True: 
                time.sleep(0.01)
        self.stop_camera()
        sys.exit()

class Tracker:
    """
    A basic color tracker, it will look for colors in a range and
    create an x and y offset value from the midpoint
    """

    def __init__(self):
        pass

    def track(self, frame, color_filter_vals_min, color_filter_vals_max): 
        hsv = cv2.GaussianBlur(frame, (11, 11), 0) 
        height = frame.shape[0]
        width = frame.shape[1]

        ink_loc_h = height*(285.0/height) # this affects where the width measurement is taken in pixels
        mm_per_pix = 0.00072/10 # to be determined from the needle diameter

        mask = cv2.inRange(hsv, (color_filter_vals_min[2], color_filter_vals_min[1], color_filter_vals_min[0]), (color_filter_vals_max[2], color_filter_vals_max[1], color_filter_vals_max[0]))
        # mask = cv2.erode(mask, None, iterations=2)
        # mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) 
        #print(contours)
        if len(contours)>0:
            for contour in contours:
                ink_contour = contour.reshape(-1,2) 
                ink_location_idx = np.where(contour.reshape(-1,2)[:,1]==ink_loc_h) 
                ink_location = ink_contour[ink_location_idx[0],:] 
                #print(ink_location) 
                #print(len(ink_location_idx[0])>1)
                if len(ink_location_idx[0])>1 and np.all(abs(ink_location[:,0]-width/2)<50): 
                    ink_width = abs(ink_location[0,0]-ink_location[1,0])*mm_per_pix
                    #print(ink_width)
                    with_contours = cv2.drawContours(frame.copy(), ink_location.reshape(-1,1,2), -1, red, 3)
                    return 0,0,mask, with_contours,ink_width
                
  
        return 0,0,mask, frame, 0

def on_low_R_thresh_trackbar(val):
    global color_filter_vals_min, color_filter_vals_max 
    color_filter_vals_min[0] = val
    color_filter_vals_min[0] = min(color_filter_vals_max[0]-1, color_filter_vals_min[0])
    cv2.setTrackbarPos(low_R_name, window_detection_name, color_filter_vals_min[0])
def on_high_R_thresh_trackbar(val):
    global color_filter_vals_min, color_filter_vals_max 
    color_filter_vals_max[0] = val
    color_filter_vals_max[0] = max(color_filter_vals_max[0], color_filter_vals_min[0]+1)
    cv2.setTrackbarPos(high_R_name, window_detection_name, color_filter_vals_max[0])
def on_low_G_thresh_trackbar(val):
    global color_filter_vals_min, color_filter_vals_max 
    color_filter_vals_min[1] = val
    color_filter_vals_min[1] = min(color_filter_vals_max[1]-1, color_filter_vals_min[1])
    cv2.setTrackbarPos(low_G_name, window_detection_name, color_filter_vals_min[1])
def on_high_G_thresh_trackbar(val):
    global color_filter_vals_min, color_filter_vals_max 
    color_filter_vals_max[1] = val
    color_filter_vals_max[1] = max(color_filter_vals_max[1], color_filter_vals_min[1]+1)
    cv2.setTrackbarPos(high_G_name, window_detection_name, color_filter_vals_max[1])
def on_low_B_thresh_trackbar(val):
    global color_filter_vals_min, color_filter_vals_max 
    color_filter_vals_min[2] = val
    color_filter_vals_min[2] = min(color_filter_vals_max[2]-1, color_filter_vals_min[2])
    cv2.setTrackbarPos(low_B_name, window_detection_name, color_filter_vals_min[2])
def on_high_B_thresh_trackbar(val):
    global color_filter_vals_min, color_filter_vals_max 
    color_filter_vals_max[2] = val
    color_filter_vals_max[2] = max(color_filter_vals_max[2], color_filter_vals_min[2]+1)
    cv2.setTrackbarPos(high_B_name, window_detection_name, color_filter_vals_max[2])


if __name__ == "__main__":
    red = (0, 0, 255)
    green = (0, 255, 0)
    blue = (255, 0, 0)
    camera_node()
