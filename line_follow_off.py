import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import math

import rospy
from clover import srv
from std_srvs.srv import Trigger

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

FLY_HEIGHT = 1

def navigate_wait(x=0, y=0, z=FLY_HEIGHT, yaw=1.57, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
    cv2.imshow("BW image", black)

    contours_blk, _ = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blk.sort(key=cv2.minAreaRect)

    if len(contours_blk) > 0:
        cnt = contours_blk[0]
        if cv2.contourArea(cnt) > 300:
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            
            box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(cv_image, [box], 0, (0,0,255),2)

            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle

            center = cv_image.shape[1] / 2
            error = x_min - center

            #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

            cv2.circle(cv_image, (int(x_min), int(y_min)), 5, (0, 0, 255), 3)

            #print(round(angle, 2), error)
            set_velocity(vx=0.2, vy=error*(-0.005), vz=0, yaw=float('nan'), yaw_rate=angle*(-0.008), frame_id='body')


    cv2.imshow("Original image", cv_image)

    cv2.waitKey(1)

print("Taking off")
navigate(x=0, y=0, z=FLY_HEIGHT, frame_id="body", speed=0.5, auto_arm=True)
rospy.sleep(5) 

print("Navigating to start point")
navigate_wait(x=0.5, y=0.5, z=FLY_HEIGHT, yaw=1.57, frame_id="aruco_map", auto_arm=False)
print("Navigated")
rospy.sleep(5)

print("Starting line")

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()