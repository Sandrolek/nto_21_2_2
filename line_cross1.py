from cv2 import contourArea
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import time
import math
import random

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

rospy.init_node('flight')

debug = rospy.Publisher('debug', Image, queue_size=1)
black_pub = rospy.Publisher('black', Image, queue_size=1)
top_pub = rospy.Publisher('top', Image, queue_size=1)
center_pub = rospy.Publisher('center', Image, queue_size=1)

bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

SPEED_X = 0.2

RIGHT_T_X = WIDTH // 2
RIGHT_T_Y = HEIGHT - 80
LEFT_T_X = WIDTH // 2 - 50
LEFT_T_Y = HEIGHT - 80

TOP_X = WIDTH // 2 - 20
TOP_Y = HEIGHT // 2 - 20

FULL_X_B = 40
FULL_X_E = WIDTH - 40

FULL_Y_B = 30
FULL_Y_E = HEIGHT - 30

turn_state = False
start_turn_time = time.time()
TIME_TURN = 1

FLY_HEIGHT = 1.1

mask = ((22, 31, 130), (38, 255, 251))

def navigate_wait(x=0, y=0, z=FLY_HEIGHT, yaw=1.57, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def convert_angle(angle=0, w_min=320, h_min=240):

    if angle < -45:
        angle = 90 + angle
    if w_min < h_min and angle > 0:
        angle = (90 - angle) * -1
    if w_min > h_min and angle < 0:
        angle = 90 + angle

    return angle

def check_right(img_r_half, orig):
    
    cnt_img_r_half, _ = cv2.findContours(img_r_half.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt_img_r_half.sort(key=cv2.minAreaRect)

    if len(cnt_img_r_half) > 0:
        cnt = cnt_img_r_half[0]
        if cv2.contourArea(cnt) > 300:
            #state = 
            
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            #print(angle)

            x_min += RIGHT_T_X

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0,0,255),2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            #print(f"angle: {angle}")

            if abs(abs(angle) - 90) < 10 or abs(angle) < 10:
                return True, [(x_min, y_min), (w_min, h_min), angle]
    
    return False

def check_left(img_l_half, orig):
    
    cnt_img_l_half, _ = cv2.findContours(img_l_half.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt_img_l_half.sort(key=cv2.minAreaRect)

    if len(cnt_img_l_half) > 0:
        cnt = cnt_img_l_half[0]
        if cv2.contourArea(cnt) > 300:
            #state = 
            
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            # print(angle)

            # x_min += RIGHT_T_X

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0,0,255),2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            #print(f"angle: {angle}")

            if abs(abs(angle) - 90) < 10 or abs(angle) < 10:
                return True, [(x_min, y_min), (w_min, h_min), angle]
    
    return False

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

    if ((abs(angle) < 10) or (abs(angle) - 90 < 10)) and ((w_min > WIDTH * 2 // 4) or (h_min > WIDTH * 2 // 4)):
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

def get_rect_center(img, orig):
    
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

            x_min += WIDTH // 2 - 30
            # y_min += FULL_Y_B

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0, 255, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            if abs(angle) < 10:
                return True, [(x_min, y_min), (w_min, h_min), angle]
            
    return False, []

turn_point = [0, 0, 0]

def image_callback(data):

    global turn_state, start_turn_time

    if turn_state:
        print("In turn state")
        if (time.time() - start_turn_time > TIME_TURN):
            turn_state = False
        else:
            return

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    cv_image = cv2.undistort(
        cv_image, np.array([[166.23942373073172,0,162.19011246829268],
        [0,166.5880923974026,109.82227735714285], [0,0,1]]), np.array([
        2.15356885e-01, -1.17472846e-01, -3.06197672e-04,-1.09444025e-04,
        -4.53657258e-03, 5.73090623e-01,-1.27574577e-01, -2.86125589e-02,
        0.00000000e+00,0.00000000e+00, 0.00000000e+00,
        0.00000000e+00,0.00000000e+00, 0.00000000e+00]),
        np.array([[166.23942373073172,0,162.19011246829268],
        [0,166.5880923974026,109.82227735714285], [0,0,1]]))

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, mask[0], mask[1])

    #black = black[FULL_Y_B:FULL_Y_E, FULL_X_B:FULL_X_E]

    black_pub.publish(bridge.cv2_to_imgmsg(black, 'mono8'))

    #cv2.imshow("BW image", black)

    dir_state = {"top":     [False, []],
                 "right":   [False, []],
                 "left":    [False, []]
                 }

    is_full, rect_full = get_rect_full(black, cv_image)

    top = black[:(HEIGHT // 2), :]
    is_top, rect_top = get_rect_top(top, cv_image)

    top_pub.publish(bridge.cv2_to_imgmsg(top, 'mono8'))

    center = black[:, (WIDTH // 2 - 30):(WIDTH // 2 + 30)]

    #is_center, rect_center = get_rect_center(center, cv_image)
    
    #center_pub.publish(bridge.cv2_to_imgmsg(center, 'mono8'))

    telem = get_telemetry()

    print(is_top)

    if not is_top and is_full:
        print("Follow LINE")
        center = cv_image.shape[1] / 2
        error = rect_full[0][0] - center

        #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

        cv2.circle(cv_image, (int(rect_full[0][0]), int(rect_full[0][1])), 5, (0, 0, 255), 3)

        #print(round(angle, 2), error)
        set_velocity(vx=SPEED_X, vy=error*(-0.005), vz=0, yaw=float('nan'), yaw_rate=rect_full[2]*(-0.008), frame_id='body')

    elif is_top:
        
        print("Found cross")

        #navigate_wait(x=telem.x+(0.5*math.cos(telem.yaw)), y=telem.y+(0.5*math.sin(telem.yaw)), z=FLY_HEIGHT, frame_id="aruco_map")
        print(f"x: {telem.x}, y: {telem.y}, yaw: {telem.yaw}")
        
        #navigate_wait(x=telem.x, y=telem.y+0.5, z=FLY_HEIGHT, frame_id="aruco_map")
        navigate(x=0.2, y=0, z=0, frame_id="body")
        
        # GET TELEM
        print("REACHED CROSS")
        rospy.sleep(3)
        navigate(x=0, y=0, z=0, speed=0.2, yaw=-1.57, frame_id="body")
        rospy.sleep(4)
        print("ROTATED")
        navigate_wait(x=telem.x+(0.5*math.cos(telem.yaw)), y=telem.y+(0.5*math.sin(telem.yaw)), z=FLY_HEIGHT, frame_id="aruco_map")
        print("GOT")

        navigate(x=telem.x, y=telem.y, z=FLY_HEIGHT, yaw=1.57, speed=0.1, frame_id="aruco_map")

    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    #cv2.imshow("Original image", cv_image)

    #cv2.waitKey(1)

print("Taking off")
navigate(x=0, y=0, z=FLY_HEIGHT, frame_id="body", speed=0.5, auto_arm=True)
rospy.sleep(5) 

print("Navigating to start point")
navigate_wait(x=1.5, y=0.5, z=FLY_HEIGHT, yaw=1.57, frame_id="aruco_map", auto_arm=False)
print("Navigated")
rospy.sleep(5)

print("Starting line")

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

rospy.spin()