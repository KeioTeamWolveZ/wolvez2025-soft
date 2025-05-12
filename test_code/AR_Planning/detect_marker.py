#距離を検出するプログラム

import cv2
import numpy as np
import cv2.aruco as aruco
from Ar_tools import Artools

# ==============================ARマーカーの設定==============================
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# マーカーサイズの設定
marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
camera_matrix = np.load("mtx.npy")
distortion_coeff = np.load("dist.npy")

# ==============================カメラの設定==============================

camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
if int(camera) == 1:
    cap = cv2.VideoCapture(1)
elif int(camera) == 2:
    from picamera2 import Picamera2 #laptopでは使わないため
    #pivcamera2/libcameraはRPiのカメラモジュールを使うためのライブラリ
    from libcamera import controls #laptopでは使わないため
    picam2 = Picamera2()
    size = (1800, 1000)
    config = picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": size})
    #カメラのフォーマット・サイズを指定

    picam2.align_configuration(config)#カメラの調整
    picam2.configure(config)
    picam2.start()#カメラの起動
    picam2.set_controls({"AfMode":0,"LensPosition":5.5})
    #AfMOde:0は固定焦点，1は連続
    #LensPosition：焦点距離を指定
    lens = 5.5#焦点位置の指定

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
    if int(camera) == 1:
        ret, frame = cap.read()
        #ret：取得できたかどうか0/1
        #frame：取得した画像
    elif int(camera) == 2:
        frame = picam2.capture_array()
    height = frame.shape[0]
    width = frame.shape[1]

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
                    TorF = ar.outlier(distance, prev, ultra_count, 0.3)
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
