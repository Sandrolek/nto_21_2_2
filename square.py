import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import time

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

mask = ((2, 85, 168), (12, 255, 255))
#mask = ((60, 120, 0), (70, 140, 255))

k = 0.25 / 2300

def image_callback(data):

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, mask[0], mask[1])
    cv2.imshow("BW image", black)

    cnts, _ = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 100:
            cv2.drawContours(cv_image, [cnt], 0, (0, 255, 0), 2)

            cnt_area = cv2.contourArea(cnt)

            S = k * cnt_area

            print(cnt_area, round(S, 2))

    cv2.imshow("Original image", cv_image)

    cv2.waitKey(1)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

rospy.spin()