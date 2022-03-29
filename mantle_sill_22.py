import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool

from pyzbar.pyzbar import decode

import numpy as np
import math

import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('find_spots')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

debug = rospy.Publisher('debug', Image, queue_size=1)
top_pub = rospy.Publisher('top', Image, queue_size=1)

bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

FLY_HEIGHT = 1

mask = ((34, 57, 61), (47, 255, 255))

k = 0.25 / 2300

SPEED_X = 0.05

def navigate_wait(x=0, y=0, z=FLY_HEIGHT, yaw=1.57, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def home(xy):    
    s = list(map(float, xy)) 
    navigate_wait(x = s[0], y= s[1], z = 2)
    rospy.sleep(3)
    while rospy.wait_for_message('rangefinder/range', Range).range > 0.3:
        navigate_wait(frame_id='body', z = -0.2, speed = 1)
        print('land')         
    if rospy.wait_for_message('rangefinder/range', Range).range <= 0.3:
        print('disarm')
        arming(False)

def convert_angle(angle=0, w_min=320, h_min=240):

    if angle < -45:
        angle = 90 + angle
    if w_min < h_min and angle > 0:
        angle = (90 - angle) * -1
    if w_min > h_min and angle < 0:
        angle = 90 + angle

    return angle

def get_qr(): 

    qr = None
    while qr == None:
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image)
    
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

        barcodes = decode(cv_image) 
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect  
            qr = barcode.data

    return qr

def get_rect_top(img, orig):
    
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    (x_min, y_min), (w_min, h_min), angle = (0, 0), (0, 0), 0

    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)

        if cv2.contourArea(cnt) > 100:
            #state = 
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            #print(angle)

            #y_min += TOP_X
            #x_min += TOP_X - 40

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (255, 0, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

    # if abs(angle) < 10:
    #     if h_min > w_min:
    #         h = h_min
    #         w = w_min
    #     else:
    #         w = 

    #print(f"Ang: {angle}, w: {w_min}, h: {h_min}")

    if (abs(angle) < 10) or (abs(angle) - 90 < 10):
        #print("TOP", random.randint(0, 100))
        return True, [(x_min, y_min), (w_min, h_min), angle]

    #print("FALSE") 
    return False, []

def get_rect_full(img, orig):
    
    cnts, _ = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    if len(cnts) > 0:
        #cnt = cnts[0]
        cnt = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(cnt) > 50:
            #state = 
            #print("FULL", random.randint(0, 100))

            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            # x_min += FULL_X_B
            # y_min += FULL_Y_B

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0, 255, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            if abs(angle) < 10:
                return True, [(x_min, y_min), (w_min, h_min), angle]
            
    return False, []

def img_cb(data):
    global state

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))
    #cv2.imshow("BW image", black)

    top = black[:(HEIGHT // 2), :]
    
    is_full, rect_full = get_rect_full(black, cv_image)
    
    is_top, rect_top = get_rect_top(top, cv_image)

    # top_pub.publish(bridge.cv2_to_imgmsg(top, 'mono8'))

    if is_full:
        print("Lining")
        (x_min, y_min), (w_min, h_min), angle = rect_full

        center = cv_image.shape[1] / 2
        error = x_min - center

        #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

        cv2.circle(cv_image, (int(x_min), int(y_min)), 5, (0, 0, 255), 3)

        #print(round(angle, 2), error)
        set_velocity(vx=SPEED_X, vy=error*(-0.005), vz=0, yaw=float('nan'), yaw_rate=angle*(-0.008), frame_id='body')
    if not is_top:
        print("Staying")

        state = True

    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    #cv2.imshow("Original image", cv_image)

    #cv2.waitKey(1)

state = False

print("Taking off")
navigate(x=0, y=0, z=1, frame_id="body", speed=1, auto_arm=True)
rospy.sleep(6)
telem = get_telemetry(frame_id="aruco_map")
navigate_wait(x=telem.x, y=telem.y, z=telem.z-0.3, frame_id="aruco_map", speed=1, auto_arm=False)
rospy.sleep(3)

start_telem = get_telemetry(frame_id="aruco_map")
S_X = start_telem.x
S_Y = start_telem.x

print("Finding QR")

res = get_qr()
start_line = list(map(float, str(res)[2:-1].split()))
print("Found QR!")
print(f"Start line is {start_line[0]} {start_line[1]}")

#navigate_wait(x=S_X, y=S_Y, z=1.5, frame_id="aruco_map")

print("Navigating to start line")
navigate_wait(x=start_line[0], y=start_line[1], z=FLY_HEIGHT, speed=0.3, frame_id="aruco_map")
rospy.sleep(2)
print("Got to start line")

print("Starting line")

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, img_cb)

while not state:

    # finding defects


    rospy.sleep(0.1)

print("Navigating to start")
navigate_wait(x=S_X, y=S_Y, z=2, frame_id="aruco_map")
rospy.sleep(2)

home([S_X, S_Y])
# home((S_X, S_Y))
