# ARマーカーを認識するプログラム
import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
from Ar_tools import Artools
import motor_pico as motor
import RPi.GPIO as GPIO
import time

from Color_tools import Color_tools

class Image:
    def __init__(self):
        pass

    def setup_AR(self):
        # ==============================ARマーカーの設定==============================
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        # マーカーサイズの設定
        self.marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
        self.camera_matrix = np.load("mtx.npy")
        self.distortion_coeff = np.load("dist.npy")
       
    def setup_color(self):
        # ==============================colorの設定=============================
        self.lower_orange = np.array([105, 56, 0])
        self.upper_orange = np.array([150, 255, 255])
        self.color_tools = Color_tools(self.lower_orange, self.upper_orange)
        self.MAX_CONTOUR_THRESHOLD = 1000
    
    def lower_color(self):
        return self.lower_orange
    def upper_color(self):
        return self.upper_orange
    def color_tools(self):
        return self.color_tools
    def MAX_CONTOUR_THRESHOLD(self):
        return self.MAX_CONTOUR_THRESHOLD
   

    def setup_camera(self, camera):
        # ==============================カメラの設定==============================
        self.cam_pint = 5.5

        # ~ camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
        self.camera = camera
        if int(self.camera) == 1:
            self.cap = cv2.VideoCapture(0)
        elif int(self.camera) == 2:
            from picamera2 import Picamera2 #laptopでは使わないため
            from libcamera import controls #laptopでは使わないため
            self.picam2 = Picamera2()  
            size = (1200, 1800)
            config = self.picam2.create_preview_configuration(
                        main={"format": 'XRGB8888', "size": size})
            self.picam2.align_configuration(config)
            self.picam2.configure(config)
            self.picam2.start()
            self.picam2.set_controls({"AfMode":0,"LensPosition":5.5})
            self.lens = 5.5

    def get_picam2(self):
        return self.picam2
    def update_image(self, camera):
            # カメラ画像の取得
            if int(self.camera) == 1:
                self.ret, self.frame = self.cap.read()
            elif int(self.camera) == 2:
                self.frame = self.picam2.capture_array()
            self.frame2 = cv2.rotate(self.frame, cv2.ROTATE_90_CLOCKWISE)
            return self.frame2

    def detect_marker(self):
            self.gray = cv2.cvtColor(self.frame2, cv2.COLOR_BGR2GRAY)# グレースケールに変換
            self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.gray, self.dictionary)# ARマーカーの検出
        
    def get_corners(self):
         return self.corners
    def get_ids(self):
            return self.ids
    def get_rejectedImgPoints(self):
            return self.rejectedImgPoints
    
    def vec_of_marker(self):
        # ==============================ARマーカーの位置==============================
        self.rvec, self.tvec, _objPoints = aruco.estimatePoseSingleMarkers(self.corners, self.marker_length, self.camera_matrix, self.distortion_coeff)
        self.tvec = np.squeeze(self.tvec)
        self.rvec = np.squeeze(self.rvec)

        # 回転ベクトルからrodoriguesへ変換
        self.rvec_matrix = cv2.Rodrigues(self.rvec)
        self.rvec_matrix = self.rvec_matrix[0] # rodoriguesから抜き出し
        self.transpose_tvec = self.tvec[np.newaxis, :].T # 並進ベクトルの転置
        self.proj_matrix = np.hstack((self.rvec_matrix, self.transpose_tvec)) # 合成
        self.euler_angle = cv2.decomposeProjectionMatrix(self.proj_matrix)[6]  # オイラー角への変換[deg]
    
    def get_rvec(self):
        return self.rvec
    def get_tvec(self):
        return self.tvec
    def get_yaw(self):
        return self.euler_angle[2]
    
    def distance(self, tvec): 
        polar_exchange = Artools().polar_change(tvec)
        return self.polar_exchange[0]
    
    def angle(self, tvec):
        polar_exchange = Artools().polar_change(tvec)
        return self.polar_exchange[1]

    
    
