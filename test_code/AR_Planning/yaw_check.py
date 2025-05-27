# ARマーカーを認識するプログラム
import numpy as np
import cv2
import Image

Img = Image.Image()
camera = 2

Img.setup_AR()
Img.setup_camera(camera)

while True:
    frame2 = Img.update_image(camera)
    picam2 = Img.get_picam2()

    frame2 = picam2.capture_array()

    Img.detect_marker()
    corners = Img.get_corners()
    ids = Img.get_ids()

    if ids is not None:
        ids = ids.tolist()
        if focus_num == 10:
            focus_num = ids[0]
            print("focus_num:",focus_num)
        for i in range(len(ids)):
            if focus_num in ids:
                k = ids.index(focus_num) 
                Img.vec_of_marker(k)
                prev = list(prev)
                rvec = Img.get_rvec()
                tvec = Img.get_tvec()
                yaw = Img.get_yaw()
                print(yaw)

        
    frame = cv2.resize(frame2,None,fx=0.3,fy=0.3)
    cv2.imshow('ARmarker', frame)
    key = cv2.waitKey(1)# キー入力の受付
    if key == 27:  # ESCキーで終了
        break
