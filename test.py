import cv2

import numpy as np

import rospy

def info(img, name):    
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))

    contours_blk, _ = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blk.sort(key=cv2.minAreaRect)

    if len(contours_blk) > 0:
        cnt = contours_blk[0]
        if cv2.contourArea(cnt) > 300:
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            print(angle)

            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle

            print(f"{name}: {x_min}, {y_min}, {round(angle, 2)}")

            box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(img, [box], 0, (0,0,255),2)

    cv2.imshow(name, img)

img = cv2.imread("_60.png")
info(img, "-60")

img = cv2.imread("_45.png")
info(img, "-45")

img = cv2.imread("_30.png")
info(img, "-30")

img = cv2.imread("0.png")
info(img, "0")

img = cv2.imread("30.png")
info(img, "30")

img = cv2.imread("45.png")
info(img, "45")

img = cv2.imread("60.png")
info(img, "60")

img = cv2.imread("90.png")
info(img, "90")

cv2.waitKey(0)