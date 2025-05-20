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
import Image

Img = Image.Image()

Img = Img.setup()

# ==============================Colorの設定===============================
lower_orange = Img.lower_color()
upper_orange = Img.upper_color()
color_tools = Img.color_tools()
MAX_CONTOUR_THRESHOLD = Img.MAX_CONTOUR_THRESHOLD()

# ==============================モーターの設定==============================
motor1 = motor.motor(dir=-1)
motor2 = motor.motor()

# ==============================定数の設定==============================
VEC_GOAL = [0.0, 0.1968730025228114, 0.3]
ultra_count = 0
reject_count = 0
prev = np.array([])
TorF = True
sdnk_pos = "Left"
plan = "Plan_A"
find_marker = False
lost_marker_cnt = 0

 # ==============================成功の定義==============================
closing_threshold = 0.4
closing_range = 0.02
k = 0
j = 0

# ==============================クラスのインスタンス化==============================
ar = Artools()


camera = 2
while True:
    frame = Img.update_image(camera)[0] #(0:frame, 1:frame2)
    frame2 = Img.update_image(camera)[1]
    # picam2.set_controls({"AfMode":0,"LensPosition":lens})
    # カメラ画像の取得
    if int(camera) == 1:
        ret, frame = cap.read()
    elif int(camera) == 2:
        frame = picam2.capture_array()
    
    height = frame2.shape[0]
    width = frame2.shape[1]

    corners = Img.get_corners()
    ids = Img.get_ids()  

    # オレンジ色のマスクを作成
    mask_orange = color_tools.mask_color(frame,lower_orange,upper_orange)
    # 輪郭を抽出して最大の面積を算出し、線で囲む
    mask_orange,cX,cY,max_contour_area = color_tools.detect_color(mask_orange,MAX_CONTOUR_THRESHOLD)
    #print("cX:",cX,"cY:",cY,"max_contour_area:",max_contour_area)
    if cX:
       cv2.circle(frame2,(width-cY,cX),30,100,-1)

    if ids is not None:
        if focus_num == 10:
            ids = ids.tolist()
            focus_num = ids[0]
            print("focus_num:",focus_num)
        # aruco.DetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            if focus_num in ids:
                k = ids.index(focus_num) 
                image_points_2d = np.array(corners[k],dtype='double')
                # print(corners[i])

                rvec = Img.get_rvec()
                tvec = Img.get_tvec()
            
                if ultra_count < 3:
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
                        #print("x : " + str(tvec[0]))
                        #print("y : " + str(tvec[1]))
                        #print("z : " + str(tvec[2]))
                        tvec[0] = tvec[0]
                        #angle_of_marker = polar_exchange[1] #theta
                        distance_of_marker = distance(tvec)
                        angle_of_marker = angle(tvec)

                        # euler_angle[2] (Yaw角) を取得する
                        yaw = Img.get_yaw()
                        print(yaw)
                        print("======",distance_of_marker)
                        
                        if tvec[0] > 0.1:
                            print("---ARマーカーが右にある---")
                            motor1.go(70)
                            motor2.back(70)
                            time.sleep(0.5)
                            motor1.stop()
                            motor2.stop()
                            time.sleep(0.5)    
                            sdnk_pos = "Left"

                        elif tvec[0] < -0.1:
                            print("---ARマーカーが左にある---")
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
                                print("---yaw角が範囲内です---")
                                if distance_of_marker >= closing_threshold+closing_range:
                                    print("---ARマーカーが遠すぎます---")
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
            else:# focus_num not in ids
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

    elif plan == "Plan_B" and not find_marker:
    #plan_AでARマーカーを認識できなかった場合、場所を変えて再びplan_Aへ
        print("Plan_B now")
        motor1.go(60)
        motor2.go(60)
        time.sleep(0.5)
        motor1.stop()
        motor2.stop()

        plan = "Plan_A"

        
        

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
    
