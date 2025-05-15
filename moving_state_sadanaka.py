#また，色を認識しているか，などを考慮していないから必要に応じて変更必要

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



# ==============================ARマーカーの設定==============================
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# マーカーサイズの設定
marker_length = 0.0215  # マーカーの1辺の長さ（メートル）
camera_matrix = np.load("mtx.npy")
distortion_coeff = np.load("dist.npy")
find_marker = False
lost_marker_cnt = 0


# ==============================colorの設定=============================
# lower_orange = np.array([105, 56, 0])
# upper_orange = np.array([150, 255, 255])
# color_tools = Color_tools(lower_orange,upper_orange)
# MAX_CONTOUR_THRESHOLD = 1000

# ==============================カメラの設定==============================
cam_pint = 5.5

# ~ camera = input("Which camera do you want to use? (laptop:1 or picamera:2): ")
camera = 2
if int(camera) == 1:
    cap = cv2.VideoCapture(0)
elif int(camera) == 2:
    from picamera2 import Picamera2 #laptopでは使わないため
    from libcamera import controls #laptopでは使わないため
    picam2 = Picamera2()
    size = (1200, 1800)
    config = picam2.create_preview_configuration(
                main={"format": 'XRGB8888', "size": size})

    picam2.align_configuration(config)
    picam2.configure(config)
    picam2.start()
    #picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
    picam2.set_controls({"AfMode":0,"LensPosition":5.5})
    lens = 5.5
# ==================================motor setting==================================
GPIO.setwarnings(False)
motor1 = motor.motor(dir=-1)#左
motor2 = motor.motor()#右
#go_value = 70
# ====================================定数の定義====================================
VEC_GOAL = [0.0,0.1968730025228114,0.3]
ultra_count = 0
reject_count = 0 # 拒否された回数をカウントするための変数
prev = np.array([])
TorF = True

# ====================================成功の定義====================================
closing_threshold = 0.4
closing_range = 0.02
k = 0
j = 0
# ==============================クラスのインスタンス化==============================
ar = Artools()

# =======================================================================
# ==============================メインループ==============================
# =======================================================================
sdnk_pos = "Left"
plan = "Plan_A"

while True:
    # picam2.set_controls({"AfMode":0,"LensPosition":lens})
    # カメラ画像の取得
    if int(camera) == 1:
        ret, frame = cap.read()
    elif int(camera) == 2:
        frame = picam2.capture_array()
    
    frame2 = cv2.rotate(frame,cv2.ROTATE_90_CLOCKWISE)
    height = frame2.shape[0]
    width = frame2.shape[1]


    gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY) # グレースケールに変換
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary) # ARマーカーの検出  
    

    # # オレンジ色のマスクを作成
    # mask_orange = color_tools.mask_color(frame,lower_orange,upper_orange)
    # # 輪郭を抽出して最大の面積を算出し、線で囲む
    # mask_orange,cX,cY,max_contour_area = color_tools.detect_color(mask_orange,MAX_CONTOUR_THRESHOLD)
    # #print("cX:",cX,"cY:",cY,"max_contour_area:",max_contour_area)
    # if cX:
    #     cv2.circle(frame2,(width-cY,cX),30,100,-1)

    if ids is not None:
        if focus_num is None:
            focus_num = ids[0]
            print("focus_num:",focus_num)
        # aruco.DetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            if ids[i] == focus_num:
                image_points_2d = np.array(corners[focus_num],dtype='double')
                # print(corners[i])

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, distortion_coeff)
                tvec = np.squeeze(tvec)
                rvec = np.squeeze(rvec)
                # 回転ベクトルからrodoriguesへ変換
                rvec_matrix = cv2.Rodrigues(rvec)
                rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
                transpose_tvec = tvec[np.newaxis, :].T # 並進ベクトルの転置
                proj_matrix = np.hstack((rvec_matrix, transpose_tvec)) # 合成
                euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # オイラー角への変換[deg]
                prev = list(prev)
                #lost_marker_cnt = 0

                if ultra_count < 20:
                    prev.append(tvec)
                    print("ARマーカーの位置を算出中")
                    ultra_count += 1 #最初（位置リセット後も）は20回取得して平均取得
                    find_marker = True
                else:
                    # print("prev_length: ",len(prev))
                    TorF = ar.outlier(tvec, prev, ultra_count, 0.3) # true:correct, false:outlier
                    ultra_count += 1
                    find_marker = True
                    if TorF: # detected AR marker is reliable
                        reject_count = 0
                        print("x : " + str(tvec[0]))
                        print("y : " + str(tvec[1]))
                        print("z : " + str(tvec[2]))
                        tvec[0] = tvec[0]
                        polar_exchange = ar.polar_change(tvec)
                        print(f"yunosu_function_{ids[i]}:",polar_exchange)
                        distance_of_marker = polar_exchange[0] #r
                        angle_of_marker = polar_exchange[1] #theta
                        # euler_angle[2] (Yaw角) を取得する
                        yaw = euler_angle[2]
                        print("======",distance_of_marker)
                        
                        if tvec[0] > 0.1:
                            motor1.go(70)
                            motor2.back(70)
                            time.sleep(0.5)
                            motor1.stop()
                            motor2.stop()
                            time.sleep(0.5)    
                            sdnk_pos = "Left"

                        elif tvec[0] < -0.1:
                            motor1.back(70)
                            motor2.go(70)
                            time.sleep(0.5)
                            motor1.stop()
                            motor2.stop()
                            time.sleep(0.5)
                            sdnk_pos = "Right"
                        
                        else:
                            if yaw > 20:
                                print("---右に回転する---")
                                rgain = (closing_threshold - distance_of_marker)/closing_threshold
                                motor1.go(40 + 50*rgain)   # 左モーターを前進
                                motor2.back(40 + 50*rgain) # 右モーターを後退
                                time.sleep(0.5)
                                motor1.stop()
                                motor2.stop()
                                time.sleep(0.5)
                                motor2.go(60)   # 左にカーブ
                                motor1.go(60 - 50*rgain) 
                                time.sleep(0.5)
                                motor1.stop()
                                motor2.stop()

                                sdnk_pos = "Left" #focus_numを見失っているとしたら左側にあるはず

                            elif yaw < -20:
                                print("---左に回転する---")
                                rgain = (closing_threshold - distance_of_marker)/closing_threshold
                                motor2.go(40 + 50*rgain)   # 右モーターを前進
                                motor1.back(40 + 50*rgain) # 左モーターを後退
                                time.sleep(0.5)
                                motor1.stop()
                                motor2.stop()

                                motor1.go(60 - 50*rgain) # 右にカーブ   
                                motor2.go(60) 
                                time.sleep(0.3)
                                motor1.stop()
                                motor2.stop()

                                sdnk_pos = "Right" #focus_numを見失っているとしたら右側にあるはず

                            else:
                                if distance_of_marker >= closing_threshold+closing_range:
                                    print("---yaw角が範囲内です---")
                                    tgain = (distance_of_marker - closing_threshold)/closing_threshold
                                    motor2.go(50+50*tgain)
                                    motor1.go(50+50*tgain)
                                    time.sleep(0.5)
                                    motor2.stop()
                                    motor1.stop()
                                
                                elif distance_of_marker >= closing_threshold-closing_range:
                                    print("---ARマーカーの位置は適正です---")
                                    break
                                
                                elif distance_of_marker < closing_threshold-closing_range:
                                    print("---ARマーカーが近すぎます---")
                                    motor1.go(100) # 反転
                                    motor2.back(100)
                                    time.sleep(0.5)
                                    motor1.stop()
                                    motor2.stop()

                                    motor1.go(30)
                                    motor2.go(30)
                                    time.sleep(0.3)
                                    motor1.stop()
                                    motor2.stop()

                                    motor1.back(100) # 反転
                                    motor2.go(100)
                                    time.sleep(0.5)
                                    motor1.stop()
                                    motor2.stop()    

                            
                    else: # detected AR marker is not reliable
                        print("state of marker is rejected")
                        find_marker = False
                        print(ultra_count)
                        reject_count += 1 # 拒否された回数をカウント
                        if reject_count > 10: # 拒否され続けたらリセットしてARマーカーの基準を上書き（再計算）
                            ultra_count = 0
                            reject_count = 0 #あってもなくても良い
    

                    # 発見したマーカーから1辺が30センチメートルの正方形を描画
                    color = (0,255,0)
                    line = np.int32(np.squeeze(corners[i]))
                    cv2.polylines(frame,[line],True,color,2)
                        
                    cv2.line(frame, (width//2,0), (width//2,height),(255,255,0))
                    distance, angle = ar.Correct(tvec,VEC_GOAL)
                    polar_exchange = ar.polar_change(tvec)
                    # print("kabuto_function:",distance,angle)
                    # print("yunosu_function:",polar_exchange)
                    change_lens = -17.2*polar_exchange[0]+9.84
                    if change_lens < 3:
                        lens = 3
                    elif change_lens > 10:
                        lens = 10.5
                    else:
                        lens = change_lens
            else:
                if sdnk_pos == "Left":
                    motor2.go(40)
                    motor1.back(40)
                    time.sleep(0.5)
                    motor1.stop()
                    motor2.stop()
                    time.sleep(0.5)

                    motor1.go(80)
                    motor2.go(60)
                    time.sleep(0.5)
                    motor1.stop()
                    motor2.stop()

                    plan = "Plan_A"
                    
                elif sdnk_pos == "Right":   
                    motor1.go(40)
                    motor2.back(40)
                    time.sleep(0.5)
                    motor1.stop()
                    motor2.stop()
                    time.sleep(0.5)

                    motor1.go(60)
                    motor2.go(80)
                    time.sleep(0.5)
                    motor1.stop()
                    motor2.stop()

                    plan = "Plan_A"

    
    elif plan == "Plan_A" and not find_marker: #ARマーカを認識していない時，認識するまでその場回転
        print("Plan_A now")
        lost_marker_cnt+=1
        if lost_marker_cnt < 10:
            if sdnk_pos == "Left":
                motor2.go(30)
                motor1.back(30)
                time.sleep(0.5)
                motor1.stop()
                motor2.stop()
            
            elif sdnk_pos == "Right":
                motor1.go(30)
                motor2.back(30)
                time.sleep(0.5)
                motor1.stop()
                motor2.stop()
        
        else:
            plan = "Plan_B"
            lost_marker_cnt = 0

    elif plan == "Plan_B" and not find_marker: #ARマーカを認識していない時，認識するまでその場回転
        print("Plan_B now")
        motor1.go(60)
        motor2.go(60)
        time.sleep(0.5)
        motor1.stop()
        motor2.stop()

        plan = "Plan_A"

        
        # if cX:
        # cam_pint = 10.5
        #     while cam_pint > 3.0: #pint change start
        #         if ids is None:
        #             cam_pint -= 0.5
        #             print("pint:",cam_pint)
        #             picam2.set_controls({"AfMode":0,"LensPosition":cam_pint})
        #             frame = picam2.capture_array()
        #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # グレースケールに変換
        #             corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary)
        #         else:
        #             break
        #     if cam_pint <= 3.5:
        #         x,y = width-cY,cX
        #         cv2.line(frame2,[width//2,10],[width//2,1200],(255,0,255),5)
        #         cv2.line(frame2,[x,10],[x,1200],(255,0,255),5)
                
        #         cam_pint = 5.5 #default pint
        #         picam2.set_controls({"AfMode":0,"LensPosition":cam_pint})
        #         if x < width/2-100:
        #             print(f"color:ARマーカー探してます(LEFT) (x={x})")
        #             motor1.back(40)   #その場左回転
        #             motor2.go(60)
        #             time.sleep(0.5)
        #             motor1.stop()
        #             motor2.stop()
        #         elif x > width/2+100:
        #             print(f"color:ARマーカー探してます(RIGHT) (x={x})")
        #             motor1.go(60)   #その場左回転
        #             motor2.go(40)
        #             time.sleep(0.5)
        #             motor1.stop()
        #             motor2.stop()
        #         else:
        #             print(f"color:ARマーカー探してます(GO) (x={x})")
        #             motor1.go(50)   #その場左回転
        #             motor2.go(50)
        #             time.sleep(0.5)
        #             motor1.stop()
        #             motor2.stop()
            
                            
        # if yunosu_pos == "Left":
        #     print("ARマーカー探してます(LEFT)")
        #     motor1.back(60)   #その場左回転
        #     motor2.go(60)
        #     time.sleep(0.5)
        #     motor1.stop()
        #     motor2.stop()
                
        # elif yunosu_pos == "Right":
        #     print("ARマーカー探してます(RIGHT)")
        #     motor1.go(60)   #その場左回転
        #     motor2.back(60)
        #     time.sleep(0.5)
        #     motor1.stop()
        #     motor2.stop()
           
        
    # elif last_pos == "Plan_B":# 進みながらARマーカーを探す
    #     lost_marker_cnt+=1
    #     print("lost marker cnt +1")
    #     if lost_marker_cnt > 10:
    #         if yunosu_pos == "Left":
    #             gain1 = 30
    #             gain2 = 0
    #         else:
    #             gain1 = 0
    #             gain2 = 30
                
    #         print("Plan_B now")
    #         motor1.go(70+gain1)
    #         motor2.go(70+gain2)
    #         time.sleep(2.5 + k)
    #         motor1.stop()
    #         motor2.stop()
    #         last_pos = "Plan_A"
    #         k += 1
    #         print(k)
            
    
    #elif cX:
    #    print("=============")
        

        # else:
        #     print("認識していません")               


    time.sleep(0.05)
    # ====================================結果の表示===================================
    # #　画像のリサイズを行う
    print("find_marker",find_marker)
    print("last_pos",plan)
    frame = cv2.resize(frame2,None,fx=0.3,fy=0.3)
    cv2.imshow('ARmarker', frame)
    key = cv2.waitKey(1)# キー入力の受付
    if key == 27:  # ESCキーで終了
        break

# ==============================終了処理==============================
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
    
