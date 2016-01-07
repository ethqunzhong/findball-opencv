# -*- encoding:utf-8 -*-
import cv2
import time

interval = 10  # seconds,delay-time
num_frames = 3  # number of frame
out_fps = 24  # 摄像头输出帧数

capture = cv2.VideoCapture(0)
size = (320, 240)
video = cv2.VideoWriter("time_lapse.avi", cv2.VideoWriter_fourcc(*'XVID'), out_fps, size)

# for low quality webcams, discard the starting unstable frames
for i in xrange(42):
    capture.read()

# capture frames to video
for i in xrange(num_frames):
    _, frame = capture.read()
    video.write(frame)
    # Optional, in case you need the frames for GIF or so
    filename = '{:4}.png'.format(i).replace(' ', '0')
    cv2.imwrite(filename, frame)

    print('Frame {} is captured.'.format(i))
    time.sleep(interval)

capture.release()
video.release()
