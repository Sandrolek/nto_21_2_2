import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge
from clover import srv
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger

import math

import numpy as np
import cv2

rospy.init_node('main')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

bridge = CvBridge()

res0 = rospy.Publisher('res0', Image, queue_size=1)
res1 = rospy.Publisher('res1', Image, queue_size=1)

yellow_mask = ((20, 80, 120), (40, 255, 255))
green_mask = ((50, 130, 200), (70, 255, 255))

cur_mask = yellow_mask

FLY_HEIGHT = 1

def navigate_wait(x=0, y=0, z=0, yaw=math.radians(90), speed=0.3, frame_id='', auto_arm=False, tolerance=0.15, func=None):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        if func:
            func()
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.1)

def normalize(x, y):
    return x / math.sqrt(x ** 2 + y ** 2), y / math.sqrt(x ** 2 + y ** 2)

def image_cb(data):

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    cur = cv2.inRange(hsv, cur_mask[0], cur_mask[1])

    part = cur[:120][:]

    contours, _ = cv2.findContours(part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    if len(contours) == 0:
        return
    
    m = cv2.moments(contours[0])

    coords = (m['m10'] / (m['m00'] + 1e-5), m['m01'] / (m['m00'] + 1e-5))

    a = math.acos(-(coords[1] - 120) / math.sqrt((coords[1] - 120) ** 2 + (coords[0] - 160) ** 2))

    #navigate_wait(x=0.1, y=0, z=0, yaw=-a, speed=0.05, frame_id="body", auto_arm=False)

    telem = get_telemetry(frame_id='aruco_map')

    x0, y0 = 320 // 2, 240 // 2

    dx, dy = normalize(coords[0] - x0, coords[1] - y0)  # get final motion vector 
    dx /= 15  # limit the speed
    dy /= 15
    dy = -dy

    set_position(x=telem.x + dx, y=telem.y + dy, z=FLY_HEIGHT, yaw=math.radians(90) - a, frame_id='aruco_map')

    cv2.circle(cv_image, tuple(map(int, coords)), 3, (0, 0, 255), -1)

    cv2.line(cv_image, (160, 120), tuple(map(int, coords)), (0, 255, 0), 2)

    cv2.line(cv_image, (160, 0), (160, 120), (255, 0, 0), 2)

    res0.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    res1.publish(bridge.cv2_to_imgmsg(part, 'mono8'))

image_sub = rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_cb)

while True:
	rospy.sleep(0.2)