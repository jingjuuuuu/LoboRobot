"""
sudo apt install libopencv-dev python3-opencv
sudo apt install -y python3-picamera2
sudo apt install python3-libcamera
sudo apt install python3-ipywidgets
"""
import cv2
import time
import numpy as np
import libcamera
from picamera2 import Picamera2

# TO-DO 
from LOBOROBOT import LOBOROBOT  # 匯入 LOBOROBOT 類別
run_time = 0
###

def getXX(img):
    ###
    global run_time
    run_time = run_time + 1
    print("run_time = ", run_time)
    ww, hh, rh, r = 640, 480, 0.7, 3
    xx1, yy1, xx2, yy2  = int(ww*0.01), int(hh*rh), int(ww*1.0), int(hh*rh)
    p1, p2, p3, p4  = [r, hh-r], [ww-r, hh-r], [xx2, yy2], [xx1, yy2]
    ###
    img1 = cv2.resize(img, (ww, hh))        # 產生 640x480 的圖
    output = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('src/src'+str(run_time)+'.png', output)             # 存成 png

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    output = cv2.dilate(output, kernel)    # 膨脹
    output = cv2.GaussianBlur(output, (5, 5), 0)   # 指定區域單位為 (5, 5)
    output = cv2.erode(output, kernel)    # 侵蝕，將白色小圓點移除
    # cv2.imwrite('gaussian.png', output)             # 存成 png
    
    output = cv2.Canny(output, 40, 200)  # 偵測邊緣
    ###
    cv2.imwrite('canny/canny'+str(run_time)+'.png', output)
    # ROI: Region Of Interest (關注區域)
    zero = np.zeros((hh, ww, 1), dtype='uint8') # ROI
    area = [p1, p2, p3, p4]                 # botton left, botton right, upper right, upper left
    pts = np.array(area)
    zone = cv2.fillPoly(zero, [pts], 255)
    ###
    cv2.imwrite('roi.png', zone)
    output1 = cv2.bitwise_and(output, zone)    # 使用 bitwise_and
    cv2.imwrite('final/final'+str(run_time)+'.png', output1)             # 存成 png
    
    HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP = 40, 30, 40
    lines = cv2.HoughLinesP(output1, 1, np.pi / 180, 
                            HOUGH_THRESHOLD, None, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP)
    done, s1, s2, b1, b2 = 0, 0, 0, 0, 0
    img2 = img1.copy()
    # s1,b1(左邊), s2,b2(右邊)
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            x1, y1, x2, y2 = l[0], l[1], l[2], l[3]
            # Calculate the slope and intercept of the line
            cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 綠色線條，寬度為 2
            cv2.imwrite('lane_detection.png', img2)
            s = (y2 - y1) / (x2 - x1 + 1e-6)       # s = slope
            b = y1 - s * x1                 # y = s * x + b
            if min(x1,x2) < 10 or max(x1, x2) > ww - 10: # 過於靠近邊緣的線條不考慮
                continue

            # 根據 x 座標區分左線道和右線道
            if s < 0 and s < s1:
                if min(x1, x2) > ww * 0.75:
                    continue
                # print("update left x", (y1-b1)/s1)
                done = done | 1
                s1, b1 = s, b 
            if s > 0 and s > s2:
                if max(x1, x2) < ww * 0.25:
                    continue
                # print("update right x", (y1-b2)/s2)
                done = done | 2
                s2, b2 = s, b

        if done == 3: # 左右線道都有偵測到
            y1, y2 = hh-r, hh-hh*0.175            

            """
            p1 = [(int)((y1-b1)/s1), (int)(y1)]     # xmin = (int)((y1-b1)/s1)
            p2 = [(int)((y2-b1)/s1), (int)(y2)]     
            p3 = [(int)((y1-b2)/s2), (int)(y1)]     # xmax = (int)((y1-b2)/s2)
            p4 = [(int)((y2-b2)/s2), (int)(y2)]
        
            zero = np.zeros((hh, ww, 3), dtype='uint8') # 關注區域
            area = [p1, p2, p4, p3]                 # botton left, upper left, upper righ, botton right
            #print(">>", p1, p2, p4, p3)            # >> [232, 797] [450, 660] [705, 660] [798, 797]
            pts = np.array(area)
            mask = cv2.fillPoly(zero, [pts], (0, 50, 0))
            img2 = cv2.addWeighted(img2,1.0, mask,1.0, 0)        
            img2 = cv2.polylines(img2, [pts], True, (0,255,255) , 2)   # 繪製多邊形
            cv2.imwrite('a4.png', img2)             # 存成 png
            """
            final_x1 = int((y1-b1)/s1)
            final_x2 = int((y1-b2)/s2)
            cv2.circle(img2, (final_x1, hh - r), 5, (0, 0, 255), -1)  # 紅色圓點標記 x1
            cv2.circle(img2, (final_x2, hh - r), 5, (255, 0, 0), -1)  # 藍色圓點標記 x2
            cv2.imwrite('lp/lane_detection_with_points'+str(run_time)+'.png', img2)
            return((final_x1, final_x2), s1, s2)
        elif done == 1: # 只有左線道有偵測到
            y1, y2 = hh-r, hh-hh*0.175
            final_x1 = int((y1-b1)/s1)
            final_x2 = int((y2-b1)/s1)
            cv2.circle(img2, (final_x1, hh - r), 5, (0, 0, 255), -1)  # 紅色圓點標記 x1
            cv2.circle(img2, (ww, hh - r), 5, (255, 0, 0), -1)  # 藍色圓點標記 x2
            cv2.imwrite('lp/lane_detection_with_points'+str(run_time)+'.png', img2)
            return((final_x1, -1), s1, 0)
        elif done == 2: # 只有右線道有偵測到
            y1, y2 = hh-r, hh-hh*0.175
            final_x1 = int((y1-b2)/s2)
            final_x2 = int((y1-b2)/s2)
            cv2.circle(img2, (0, hh - r), 5, (0, 0, 255), -1)  # 紅色圓點標記 x1
            cv2.circle(img2, (final_x2, hh - r), 5, (255, 0, 0), -1)  # 藍色圓點標記 x2
            cv2.imwrite('lp/lane_detection_with_points'+str(run_time)+'.png', img2)
            return((-1, final_x2), 0, s2)
    # 沒有偵測到車道線
    return((-1, -1), 0, 0)    

picamera = Picamera2()
config = picamera.create_preview_configuration(
                main={"format": 'RGB888', "size": (640, 480)},
                raw={"format": "SRGGB12", "size": (640, 480)}) 
                # raw={"format": "SRGGB12", "size": (1920, 1080)}
config["transform"] = libcamera.Transform(hflip=0, vflip=1)
picamera.configure(config)
picamera.start()
# TO-DO
clbrobot = LOBOROBOT()
###
i = 0
t0 = time.time()
t1 = t0
missed_frames = 0
while True:
    try:
        img = picamera.capture_array()
        # 左右翻轉圖片
        img = cv2.flip(img, 1)  # 第二個參數 1 表示水平翻轉
        
        # 假設車子開在道路中，取得左、右車道線 X 座標
        ((x1, x2), s1, s2) = getXX(img)
        if x1 < 0:
            x1 = -1
            s1 = 0
        if x2 > 640:
            x2 = -1
            s2 = 0
        print("output (x1, x2) = ", (x1, x2))
        print("output (s1, s2) = ", (s1, s2))
        # break
        
        # 控制你車子要直走、偏右走、或邊左走！
        # TO-DO
        
        if x1 == -1 and x2 == -1:
            # clbrobot.t_stop(0)
            missed_frames += 1
            print(f"連續未偵測次數: {missed_frames}")
            
            # 如果超過容許的未偵測幀數，停止車輛
            if missed_frames > 10:
                print("連續未偵測到車道線，停止車輛")
                clbrobot.t_stop(0)
                break
            else:
                # 減速或短暫停止
                clbrobot.t_up(20, 0.2)
                continue
        else:
            missed_frames = 0  # 重置未偵測計數

        road_center = (x1 + x2) // 2
        image_center = img.shape[1] // 2
        offset = road_center - image_center # 畫面中央與道路中央偏移量
        print("offset = ", offset)
        ####小轉
        if x1 != -1 and s1 > -0.9 and s1 < -0.78 : # 左線斜率介於-0.78到-0.9之間要小右轉
            print("小右轉")
            if x1 < 100:
                clbrobot.t_up(30, 1)
                clbrobot.turnRight(30, 0.3)
            else:
                clbrobot.t_up(30, 1)
                clbrobot.turnRight(30, 0.3)
            clbrobot.t_stop(0.15)
        elif x2 != -1 and s2 < 0.9 and s2 > 0.78: # 右線斜率介於0.78到0.9之間要小左轉
            print("小左轉")
            if x2 > 540:
                clbrobot.t_up(30, 1)
                clbrobot.turnLeft(30, 0.3)
            else:
                clbrobot.t_up(30, 1)
                clbrobot.turnLeft(30, 0.3)
            clbrobot.t_stop(0.15)
        if x1 != -1 and s1 > -0.78 and s1 < -0.45 : # 左線斜率介於-0.78到-0.45之間要右轉
            print("右轉")
            if x1 < 100:
                clbrobot.t_up(30, 1)
                clbrobot.turnRight(30, 0.45)
            else:
                clbrobot.t_up(30, 1)
                clbrobot.turnRight(30, 0.45)
            clbrobot.t_stop(0.15)
        elif x2 != -1 and s2 < 0.78 and s2 > 0.45: # 右線斜率介於0.78到0.45之間要左轉
            print("左轉")
            if x2 > 540:
                clbrobot.t_up(30, 1)
                clbrobot.turnLeft(30, 0.45)
            else:
                clbrobot.t_up(30, 1)
                clbrobot.turnLeft(30, 0.45)
            clbrobot.t_stop(0.15)
        ####大轉
        elif x1 != -1 and s1 > -0.45:  # 左線斜率大於-0.45，要大右轉
            print("大右轉")
            if x1 < 100:
                clbrobot.t_up(30, 0.95)
                clbrobot.turnRight(30, 0.6)
            else:
                clbrobot.t_up(30, 0.95)
                clbrobot.turnRight(30, 0.6)
            clbrobot.t_stop(0.15)
        elif x2 != -1 and s2 < 0.45: # 右線斜率小於0.45，要大左轉
            print("大左轉")
            if x2 > 540:
                clbrobot.t_up(30, 0.95)
                clbrobot.turnLeft(30, 0.6)
            else:
                clbrobot.t_up(30, 0.95)
                clbrobot.turnLeft(30, 0.6)
            clbrobot.t_stop(0.15)
        #### 微調邏輯 ####
        else:
            # 如果只偵測到右線或左線
            if x1 == -1 or x2 == -1:  
                if x1 == -1 and x2 < 500:  # 偵測到右線，偏右，微調向左
                    print("偏右，微調向左")
                    clbrobot.turnLeft(20, 0.1)  # 車輛左轉，速度 20，持續 0.1 秒
                elif x2 == -1 and x1 > 140:  # 偵測到左線，偏左，微調向右
                    print("偏左，微調向右")
                    clbrobot.turnRight(20, 0.1)  # 車輛右轉，速度 20，持續 0.1 秒
                else:  # 否則保持直行
                    print("保持直行")
                    clbrobot.t_up(30, 0.2)  # 車輛向前行駛，速度 30，持續 0.2 秒
            # 根據偏移量調整車輛方向
            elif offset > 45:  # 偏左，微調向右
                print("偏左，微調向右")
                clbrobot.turnRight(20, 0.1)  # 車輛右轉，速度 20，持續 0.1 秒
            elif offset < -45:  # 偏右，微調向左
                print("偏右，微調向左")
                clbrobot.turnLeft(20, 0.1)  # 車輛左轉，速度 20，持續 0.1 秒
            else:  # 保持直行
                print("保持直行")
                clbrobot.t_up(30, 0.2)
        ###
        
        t2 = time.time()
        if t2-t1 >= 1.0:
            t1 = t2
            print("FPS:", 60/(t2-t0))       # FPS: 14.544587830627131
            
    except KeyboardInterrupt:
        clbrobot.t_stop(0)
        break

