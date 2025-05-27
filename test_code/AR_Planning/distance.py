#距離を検出するプログラム

import cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from collections import deque
from Ar_tools import Artools
import motor
import RPi.GPIO as GPIO
import time
from Image import Image

image = Image()
image.setup_AR()
camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
image.camera(camera)

# ====================================定数の定義====================================
VEC_GOAL = [0.0,0.1968730025228114,0.3]
ultra_count = 0
reject_count = 0 # 拒否された回数をカウントするための変数
prev = np.array([])
TorF = True

# ==============================クラスのインスタンス化==============================
ar = Artools()

# =======================================================================
# ==============================メインループ==============================
# =======================================================================

while True:
    picam2.set_controls({"AfMode":0,"LensPosition":lens})
    # カメラ画像の取得
    frame = image.updata_image(camera)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # グレースケールに変換
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary) # ARマーカーの検出（四隅の座標，arのid，辞書にないid）

    if ids is not None:
        for i in range(len(ids)):
            if ids[i] in [0,1,2,3,4,5]:#6面体のマーカーを認識
                tvec= aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distortion_coeff)[1]
                #rvecは回転ベクトル，tvecは並進ベクトル
                tvec = np.squeeze(tvec)#一次元にする
                distance = np.linalg.norm(tvec)
                prev= list(prev)

                if ultra_count < 20:
                    prev.append(distance)
                    print("ARマーカーの位置を算出中")
                    ultra_count += 1
                else:
                    TorF = ar.outlier_dis(distance, prev, ultra_count, 0.3)
                    ultra_count += 1
                    if TorF:
                        reject_count = 0
                        print("distance : ", distance)
                    else:
                        print("state of marker is rejected")
                        reject_count += 1
                        if reject_count > 10: # 拒否され続けたらリセットしてARマーカーの基準を上書き（再計算）
                            ultra_count = 0
                            reject_count = 0

    # 終了条件（例: 'q' キーを押す）
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break

