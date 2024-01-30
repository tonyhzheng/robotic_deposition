#!/usr/bin/env python3
import numpy as np
import cv2
import time
import pickle
import os

from threading import Thread
import cv2, time

import threading

# CAP_PROP_POS_MSEC       =0, //!< Current position of the video file in milliseconds.
# CAP_PROP_POS_FRAMES     =1, //!< 0-based index of the frame to be decoded/captured next.
# CAP_PROP_POS_AVI_RATIO  =2, //!< Relative position of the video file: 0=start of the film, 1=end of the film.
# CAP_PROP_FRAME_WIDTH    =3, //!< Width of the frames in the video stream.
# CAP_PROP_FRAME_HEIGHT   =4, //!< Height of the frames in the video stream.
# CAP_PROP_FPS            =5, //!< Frame rate.
# CAP_PROP_FOURCC         =6, //!< 4-character code of codec. see VideoWriter::fourcc .
# CAP_PROP_FRAME_COUNT    =7, //!< Number of frames in the video file.
# CAP_PROP_FORMAT         =8, //!< Format of the %Mat objects returned by VideoCapture::retrieve().
# CAP_PROP_MODE           =9, //!< Backend-specific value indicating the current capture mode.
# CAP_PROP_BRIGHTNESS    =10, //!< Brightness of the image (only for those cameras that support).
# CAP_PROP_CONTRAST      =11, //!< Contrast of the image (only for cameras).
# CAP_PROP_SATURATION    =12, //!< Saturation of the image (only for cameras).
# CAP_PROP_HUE           =13, //!< Hue of the image (only for cameras).
# CAP_PROP_GAIN          =14, //!< Gain of the image (only for those cameras that support).
# CAP_PROP_EXPOSURE      =15, //!< Exposure (only for those cameras that support).
# CAP_PROP_CONVERT_RGB   =16, //!< Boolean flags indicating whether images should be converted to RGB.
# CAP_PROP_WHITE_BALANCE_BLUE_U =17, //!< Currently unsupported.
# CAP_PROP_RECTIFICATION =18, //!< Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently).
# CAP_PROP_MONOCHROME    =19,
# CAP_PROP_SHARPNESS     =20,
# CAP_PROP_AUTO_EXPOSURE =21, //!< DC1394: exposure control done by camera, user can adjust reference level using this feature.
# CAP_PROP_GAMMA         =22,
# CAP_PROP_TEMPERATURE   =23,
# CAP_PROP_TRIGGER       =24,
# CAP_PROP_TRIGGER_DELAY =25,
# CAP_PROP_WHITE_BALANCE_RED_V =26,
# CAP_PROP_ZOOM          =27,
# CAP_PROP_FOCUS         =28,
# CAP_PROP_GUID          =29,
# CAP_PROP_ISO_SPEED     =30,
# CAP_PROP_BACKLIGHT     =32,
# CAP_PROP_PAN           =33,
# CAP_PROP_TILT          =34,
# CAP_PROP_ROLL          =35,
# CAP_PROP_IRIS          =36,
# CAP_PROP_SETTINGS      =37, //!< Pop up video/camera filter dialog (note: only supported by DSHOW backend currently. The property value is ignored)
# CAP_PROP_BUFFERSIZE    =38,
# CAP_PROP_AUTOFOCUS     =39,
# CAP_PROP_SAR_NUM       =40, //!< Sample aspect ratio: num/den (num)
# CAP_PROP_SAR_DEN       =41, //!< Sample aspect ratio: num/den (den)

# bufferless VideoCapture
class VideoCaptureBufferless:
    # https://stackoverflow.com/questions/43665208/how-to-get-the-latest-frame-from-capture-device-camera-in-opencv
    def __init__(self, src=0, show_window = True):
        self.video = cv2.VideoCapture(src)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) # depends on fourcc available camera
        print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))) 
        # self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) 
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160) 
        self.video.set(cv2.CAP_PROP_AUTOFOCUS, 0) #turnsoff autofocus 
        self.video.set(cv2.CAP_PROP_BRIGHTNESS, 161)
        self.video.set(cv2.CAP_PROP_CONTRAST, 228)
        self.video.set(cv2.CAP_PROP_SATURATION, 255)
        self.video.set(cv2.CAP_PROP_SHARPNESS, 110)
        self.video.set(cv2.CAP_PROP_FOCUS , 21)
        self.video.set(cv2.CAP_PROP_FPS, 30) 
        self.video.set(cv2.CAP_PROP_EXPOSURE , 860)

        print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CAP_PROP_FPS : '{}'".format(self.video.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_POS_MSEC : '{}'".format(self.video.get(cv2.CAP_PROP_POS_MSEC)))
        print("CAP_PROP_FRAME_COUNT  : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_COUNT)))
        print("CAP_PROP_BRIGHTNESS : '{}'".format(self.video.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST : '{}'".format(self.video.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION : '{}'".format(self.video.get(cv2.CAP_PROP_SATURATION)))
        print("CAP_PROP_HUE : '{}'".format(self.video.get(cv2.CAP_PROP_HUE)))
        print("CAP_PROP_GAIN  : '{}'".format(self.video.get(cv2.CAP_PROP_GAIN)))
        print("CAP_PROP_CONVERT_RGB : '{}'".format(self.video.get(cv2.CAP_PROP_CONVERT_RGB)))
        print("CAP_PROP_EXPOSURE : '{}'".format(self.video.get(cv2.CAP_PROP_EXPOSURE)))


        width = 1280
        height = 720
        if show_window:
            cv2.namedWindow('finalImg', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('finalImg', width, height)

        self.lock = threading.Lock()
        self.t = threading.Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

        with open(os.path.dirname(os.path.abspath(__file__))+'/camera_params.pkl', 'rb') as f:
            camera_params = pickle.load(f)


        self.mtx = camera_params['mtx']
        self.dist = camera_params['dist']
        self.rvecs = camera_params['rvecs']
        self.tvecs = camera_params['tvecs'] 

        w = int(self.video.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h)) 

    # grab frames as soon as they are available
    def _reader(self):
        while True:
            with self.lock:
                ret = self.video.grab()
            if not ret:
                break

    # retrieve latest frame
    def read(self):
        with self.lock:
            _, frame = self.video.retrieve()
        return frame

    def show_frame(self, wait_time=1):
        # Display frames in main program
        image = self.read()
        image = cv2.undistort(image, self.mtx, self.dist, None, self.newcameramtx)  
        x, y, w, h = self.roi
        image = image[y:y+h, x:x+w] 
        cv2.imshow('finalImg', image)
        key = cv2.waitKey(wait_time)
        if key == ord('q'):
            self.video.release()
            cv2.destroyAllWindows()
            exit(1)
        
    def stream_video(self):
        while True:
            try:
                time_prev = time.time() 
                self.show_frame()
                print(time.time()-time_prev,1/(time.time()-time_prev))
            except AttributeError:
                pass 

    def display_latest(self): 
        # time_prev = time.time() 
        self.show_frame(wait_time=1)
        # print(time.time()-time_prev,1/(time.time()-time_prev))
            
    def take_image(self, file_name = None):
        time.sleep(2)
        image = self.read()
        image = cv2.undistort(image, self.mtx, self.dist, None, self.newcameramtx)  
        x, y, w, h = self.roi
        image = image[y:y+h, x:x+w] 
        if file_name: 
            # image = cv2.undistort(image, self.mtx, self.dist, None, self.newcameramtx)
            if os.path.isfile(file_name):
                result=cv2.imwrite(file_name[:-4]+'_2'+'.jpg', image)
            else:
                result=cv2.imwrite(file_name, image)
        return image

class VideoStreamWidget(object):
    def __init__(self, src=0):
        # https://stackoverflow.com/questions/54933801/how-to-increase-performance-of-opencv-cv2-videocapture0-read
        self.video = cv2.VideoCapture(src)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) # depends on fourcc available camera
        print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))) 
        # self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) 
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160) 
        self.video.set(cv2.CAP_PROP_AUTOFOCUS, 0) #turnsoff autofocus 
        self.video.set(cv2.CAP_PROP_BRIGHTNESS, 128)
        self.video.set(cv2.CAP_PROP_CONTRAST, 128)
        self.video.set(cv2.CAP_PROP_SATURATION, 255)
        self.video.set(cv2.CAP_PROP_FOCUS , 25)
        self.video.set(cv2.CAP_PROP_FPS, 30) 
        self.video.set(cv2.CAP_PROP_EXPOSURE , 860)

        print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CAP_PROP_FPS : '{}'".format(self.video.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_POS_MSEC : '{}'".format(self.video.get(cv2.CAP_PROP_POS_MSEC)))
        print("CAP_PROP_FRAME_COUNT  : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_COUNT)))
        print("CAP_PROP_BRIGHTNESS : '{}'".format(self.video.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST : '{}'".format(self.video.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION : '{}'".format(self.video.get(cv2.CAP_PROP_SATURATION)))
        print("CAP_PROP_HUE : '{}'".format(self.video.get(cv2.CAP_PROP_HUE)))
        print("CAP_PROP_GAIN  : '{}'".format(self.video.get(cv2.CAP_PROP_GAIN)))
        print("CAP_PROP_CONVERT_RGB : '{}'".format(self.video.get(cv2.CAP_PROP_CONVERT_RGB)))
        print("CAP_PROP_EXPOSURE : '{}'".format(self.video.get(cv2.CAP_PROP_EXPOSURE)))


        width = 1280
        height = 720
        cv2.namedWindow('finalImg', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('finalImg', width, height)


        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.video.isOpened():
                (self.status, self.frame) = self.video.read()
            time.sleep(.01)

    def show_frame(self):
        # Display frames in main program
        cv2.imshow('finalImg', self.frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.video.release()
            cv2.destroyAllWindows()
            exit(1)

    def stream_video(self):
        while True:
            try:
                time_prev = time.time() 
                self.show_frame()
                print(time.time()-time_prev,1/(time.time()-time_prev))
            except AttributeError:
                pass 
    
    def display_latest(self):
        while True:
            input()
            time_prev = time.time() 
            self.show_frame()
            print(time.time()-time_prev,1/(time.time()-time_prev))
            
class camera():
    def __init__(self, ): 
        self.initialize_camera()

    def initialize_camera(self):
        self.video = cv2.VideoCapture(0)
        # self.video = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) # depends on fourcc available camera
        print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))) 
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) 
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160) 
        # self.video.set(cv2.CAP_PROP_AUTOFOCUS, 0) #turnsoff autofocus 
        self.video.set(cv2.CAP_PROP_BRIGHTNESS, 128)
        self.video.set(cv2.CAP_PROP_CONTRAST, 255)
        self.video.set(cv2.CAP_PROP_SATURATION, 255)
        # self.video.set(cv2.CAP_PROP_FOCUS , 25)
        self.video.set(cv2.CAP_PROP_FPS, 30) 
        # self.video.set(cv2.CAP_PROP_EXPOSURE , 860)

        print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CAP_PROP_FPS : '{}'".format(self.video.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_POS_MSEC : '{}'".format(self.video.get(cv2.CAP_PROP_POS_MSEC)))
        print("CAP_PROP_FRAME_COUNT  : '{}'".format(self.video.get(cv2.CAP_PROP_FRAME_COUNT)))
        print("CAP_PROP_BRIGHTNESS : '{}'".format(self.video.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST : '{}'".format(self.video.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION : '{}'".format(self.video.get(cv2.CAP_PROP_SATURATION)))
        print("CAP_PROP_HUE : '{}'".format(self.video.get(cv2.CAP_PROP_HUE)))
        print("CAP_PROP_GAIN  : '{}'".format(self.video.get(cv2.CAP_PROP_GAIN)))
        print("CAP_PROP_CONVERT_RGB : '{}'".format(self.video.get(cv2.CAP_PROP_CONVERT_RGB)))
        print("CAP_PROP_EXPOSURE : '{}'".format(self.video.get(cv2.CAP_PROP_EXPOSURE)))
        # sys.exit()

        with open(os.path.dirname(os.path.abspath(__file__))+'/camera_params.pkl', 'rb') as f:
            camera_params = pickle.load(f)


        self.mtx = camera_params['mtx']
        self.dist = camera_params['dist']
        self.rvecs = camera_params['rvecs']
        self.tvecs = camera_params['tvecs'] 

        w = int(self.video.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, self.newcameramtx, (w, h), cv2.CV_32FC1)
        # map1, map2=cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, frame_size, cv2.CV_32FC1)
        
        # # camera intrinsics for height = 0.148m, autofocus 25
        # self.mtx = np.array([[1.27546226e+04, 0.00000000e+00, 1.92981648e+03],
        #     [0.00000000e+00, 1.26941631e+04, 1.07227768e+03],
        #     [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        # self.dist = np.array([[ 5.33987045e+00, -3.97330659e+02,  3.71004086e-03,
        #         3.32599921e-03,  6.58868517e+03]])

        # self.newcameramtx = np.array([[12487.6377 ,     0.     ,  1932.81387],
        #     [    0.     , 12420.20215,  1073.97054],
        #     [    0.     ,     0.     ,     1.     ]])

    
        width = 1280
        height = 720
        cv2.namedWindow('finalImg', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('finalImg', width, height)
        cv2.namedWindow('finalImg2', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('finalImg2', width, height)
        
        self.m_per_pixel_ratio = 5.072474527358006e-05

    def take_image(self, file_name = None):
        time.sleep(2)
        for k in range(50):
            success, image = self.video.read()
        if file_name: 
            # image = cv2.undistort(image, self.mtx, self.dist, None, self.newcameramtx)
            result=cv2.imwrite(file_name, image)
        return image

    def stream_video(self, undistort = True):
        time_prev = time.time()
        while True:
            time_prev = time.time()
            success, image = self.video.read()
            print(time.time()-time_prev,1/(time.time()-time_prev))
            time_prev = time.time()
            if undistort:
                image1 = cv2.undistort(image, self.mtx, self.dist, None, self.newcameramtx)  
                x, y, w, h = self.roi
                image1 = image1[y:y+h, x:x+w] 

                image2 = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
                x, y, w, h = self.roi
                image2 = image2[y:y+h, x:x+w] 
            print(time.time()-time_prev)
            print(np.sum(cv2.subtract(image[y:y+h, x:x+w] ,image1)))
            # print(np.where(cv2.subtract(image1,image2)>0))
            cv2.imshow('finalImg',image1)
            cv2.imshow('finalImg2',image)
            cv2.waitKey(1)  
if __name__ == "__main__": 
    camera()
