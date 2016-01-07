#!usr/bin/python
# -*-encoding:utf-8 -*-
"""
__author__ = 'this.zyq'
version:1.1
"""
import numpy as np
import cv2
import math
import sys
import time
import datetime

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240

GREEN_PIN = 5
RED_PIN = 4

# 设置黄色乒乓球颜色范围
HSV_MIN = np.array((15, 100, 100))
HSV_MAX = np.array((45, 255, 255))


# In study room

def noting(x):
    pass


#
# HSV_MIN = np.array((Hlow, Slow, Vlow))
# HSV_MAX = np.array((Hhigh, Shigh, Vhigh))

# 写一个滑动块动态调节阈值变化参数
# HSV_MIN = np.array((18, 85, 85))
# HSV_MAX = np.array((43, 255, 255))

# outdoor
# HSV_MIN = np.array((20, 90, 90))
# HSV_MAX = np.array((43, 255, 255))




def find_ball(capture, noimage, nothreshold):
    global HSV_MIN
    global HSV_MAX

    # cv2.createTrackbar('Hl', 'thresholded image', 15, 255, noting)
    # cv2.createTrackbar('Sl', 'thresholded image', 90, 255, noting)
    # cv2.createTrackbar('Vl', 'thresholded image', 90, 255, noting)
    # cv2.createTrackbar('Hh', 'thresholded image', 45, 255, noting)
    # cv2.createTrackbar('Sh', 'thresholded image', 200, 255, noting)
    # cv2.createTrackbar('Vh', 'thresholded image', 200, 255, noting)
    # Hlow = cv2.getTrackbarPos('Hl', 'thresholded image')
    # Slow = cv2.getTrackbarPos('Sl', 'thresholded image')
    # Vlow = cv2.getTrackbarPos('Vl', 'thresholded image')
    # Hhigh = cv2.getTrackbarPos('Hh', 'thresholded image')
    # Shigh = cv2.getTrackbarPos('Sh', 'thresholded image')
    # Vhigh = cv2.getTrackbarPos('Vh', 'thresholded image')
    #
    # HSV_MIN = np.array((int(Hlow), int(Slow), int(Vlow)))
    # HSV_MAX = np.array((int(Hhigh), int(Shigh), int(Vhigh)))
    Cx, Cy = 0, 0
    maxdiag = 0  # 判断面积
    fps = 30  #

    videoout = cv2.VideoWriter('findtheball.avi', cv2.VideoWriter_fourcc(*'XVID'), fps, (640, 480))
    ret, frame = capture.read()  # 获取每一帧
    text = "searching golf..."

    frame = cv2.resize(frame, (0, 0), fx=0.8, fy=0.8)  # set window's size

    if frame is not None:
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # convert BGR to HSV
        # 色度，饱和度，明度（纯度）
        # hsv_frame = cv2.GaussianBlur(hsv_frame, (21,21), 0) # 高斯滤波
        h, s, v = cv2.split(hsv_frame)
        thresholded = cv2.inRange(hsv_frame, HSV_MIN, HSV_MAX)  # inrange函数二值化

        # 开运算，消除白噪声
        thresholded = cv2.erode(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        thresholded = cv2.dilate(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        # 闭运算，消除前景内部空洞或者小黑点
        thresholded = cv2.dilate(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        thresholded = cv2.erode(thresholded, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

        image, contours, hierarchy = cv2.findContours(thresholded.copy(),
                                                      cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pass  # version2.0加入圆检测
        # circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, 20, param1=50,
        #                            param2=50, minRadius=0, maxRadius=0)
        # # circles=np.uint16(np.around(circles))
        # # for i in circles[0,:]:
        # if circles is not None:
        #     # convert the (x, y) coordinates and radius of the circles to integers
        #     circles = np.round(circles[0, :]).astype("int")
        #
        #     # loop over the (x, y) coordinates and radius of the circles
        #     for (x, y, r) in circles:
        #         # draw the circle in the output image, then draw a rectangle
        #         # corresponding to the center of the circle
        #         cv2.circle(frame, (x, y), r, (0, 255, 255), 4)
        #         cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

        for cnt in contours:

            if cv2.contourArea(cnt) < 100:  # 76
                continue

            x, y, w, h = cv2.boundingRect(cnt)  # （x,y）轮廓左上角坐标，（w,h）轮廓的宽和高
            cx, cy = x + w / 2, y + h / 2
            current_diag = math.sqrt(w * w + h * h) / 2
            current_diag = current_diag / (math.sqrt(2))
            if (current_diag > maxdiag):
                maxdiag, Cx, Cy = 1.384 * current_diag, int(1.023 * cx), int(1.06 * cy)  # change

        if maxdiag > 0:
            cv2.circle(frame, (Cx, Cy), int(maxdiag), (0, 255, 0), 3)
            #
            #     (x, y), radius = cv2.minEnclosingTriangle(cnt)
            #     current_diag=int (radius)
            #     if(current_diag>maxdiag):
            #         maxdiag,Cx,Cy=1.384*current_diag,int(1.023*x),int(1.06*y)
            #
            # if maxdiag>0:
            #     cv2.circle(frame,(Cx,Cy),int(maxdiag),(0,255,0),3)

            # 用户交互界面
            text = "completed! Find the ball!"
        cv2.putText(frame, "Work-Status: {}".format(text), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
                    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        if noimage == False:
            cv2.imshow("h-image", h)
            cv2.imshow("s-image",s)
            cv2.imshow("v-image",v)
            cv2.imshow("original image", frame)

        if nothreshold == False:
            cv2.imshow("thresholded image", thresholded)

        if ret is True:
            frame = cv2.flip(frame, 0)
            videoout.write(frame)

        key = cv2.waitKey(1) & 0xff
        if key == 27:
            videoout.release()
            capture.release()
            cv2.destroyAllWindows()

    else:
        print "Cannot get frame"

    return (maxdiag, Cx, Cy)  # 圆的半径和圆点坐标


def main():
    noimage = False
    nothreshold = False

    print('Initialize Camera...')  # ,
    capture = cv2.VideoCapture(0)

    if capture is not None:
        print('OK...')
    else:
        return

    # 显示小球在摄像头区域坐标信息和帧数
    frames = 0
    start_time = time.time()
    while True:
        Radius, center_x, center_y = find_ball(capture, noimage, nothreshold)

        frames += 1
        currtime = time.time()
        numsecs = currtime - start_time
        fps = frames / numsecs  # 每秒帧数

        sys.stdout.write("\t\t\r")
        sys.stdout.write("Found the ball at:(x:%d,y:%d)  Radius=%d" % (center_x, center_y, Radius) +
                         "   fps=%d" % fps)
        sys.stdout.flush()

    return


if __name__ == '__main__':
    sys.exit(main())
