import cv2
import numpy as np
import sys
import os
import psutil

vidcap = cv2.VideoCapture(0)
success,imagedisp = vidcap.read()
height, width, layers = imagedisp.shape
size = (width,height)
out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 30, size)
count = 0
try:
    print(f"Video cekimine baslandi.")
    img_array = []
    size = (0,0)
    while success:
        """hsv = cv2.cvtColor(imagedisp, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        lower_blue = np.array([100,50,50])
        upper_blue = np.array([130,255,255])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange (hsv, lower_blue, upper_blue)
        bluecnts = cv2.findContours(mask.copy(),
                              cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_SIMPLE)[-2]
        x,y=0,0
        if len(bluecnts)>0:
	        blue_area = max(bluecnts, key=cv2.contourArea)
	        (xg,yg,wg,hg) = cv2.boundingRect(blue_area);x,y=xg,yg;cv2.rectangle(imagedisp,(xg,yg),(xg+wg, yg+hg),(36,255,12),2)
        cv2.putText(imagedisp, 'Su Alma Noktasi', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
        lower_red = np.array([160,50,50])
        upper_red = np.array([180,255,255])
        # Threshold the HSV image to get only red colors
        mask = cv2.inRange (hsv, lower_red, upper_red)
        redcnts = cv2.findContours(mask.copy(),
                              cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_SIMPLE)[-2]
        x,y=0,0
        if len(redcnts)>0:
	        red_area = max(redcnts, key=cv2.contourArea)
	        (xg,yg,wg,hg) = cv2.boundingRect(red_area);x,y=xg,yg
	        cv2.rectangle(imagedisp,(xg,yg),(xg+wg, yg+hg),(36,255,12),2)
        cv2.putText(imagedisp, 'Su Birakma Noktasi', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)"""
         
        
        height, width, layers = imagedisp.shape
        
        size = (width,height)
        
        img_array.append(imagedisp)
        
        print(psutil.Process(os.getpid()).memory_info().rss / 1024 ** 2)
        if psutil.Process(os.getpid()).memory_info().rss / 1024 ** 2 > 2560: 
            for i in range(len(img_array)):
                out.write(img_array[i])
            img_array = []
        
        success,imagedisp = vidcap.read()
except KeyboardInterrupt:
    print(f"\nVideo cekimi bitirildi.")
 
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()
    print(f"\nVideo kaydedildi.")
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)