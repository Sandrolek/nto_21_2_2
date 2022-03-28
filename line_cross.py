import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import time
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

rospy.init_node('flight')

debug = rospy.Publisher('debug', Image, queue_size=1)
black_pub = rospy.Publisher('black', Image, queue_size=1)

bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

SPEED_X = 0.1

RIGHT_T_X = WIDTH // 2
RIGHT_T_Y = HEIGHT - 80
LEFT_T_X = WIDTH // 2 - 50
LEFT_T_Y = HEIGHT - 80

TOP_X = WIDTH // 2 - 20
TOP_Y = HEIGHT // 2 - 20

turn_state = False
start_turn_time = time.time()
TIME_TURN = 1

FLY_HEIGHT = 1.1

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

def check_top(img_top_half, orig):
    
    cnts, _ = cv2.findContours(img_top_half.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    if len(cnts) > 0:
        cnt = cnts[0]
        if cv2.contourArea(cnt) > 300:
            #state = 
            
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            #print(angle)

            #y_min += TOP_X
            x_min += TOP_X - 40

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0, 255, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            #print(f"angle: {angle}")

            if abs(angle) < 10:
                return True, [(x_min, y_min), (w_min, h_min), angle]
            
    return False

def check_center(img_center, orig):
    
    cnts, _ = cv2.findContours(img_center.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts.sort(key=cv2.minAreaRect)

    if len(cnts) > 0:
        cnt = cnts[0]
        if cv2.contourArea(cnt) > 300:
            #state = 
            
            rect = cv2.minAreaRect(cnt)
            (x_min, y_min), (w_min, h_min), angle = rect

            #print(angle)

            #y_min += TOP_X
            x_min += TOP_X - 40

            box = cv2.boxPoints(((x_min, y_min), (w_min, h_min), angle)) # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(orig, [box], 0, (0, 255, 0), 2)

            angle = convert_angle(angle=angle, w_min=w_min, h_min=h_min)

            #print(f"angle: {angle}")

            if abs(angle) < 10:
                return True, [(x_min, y_min), (w_min, h_min), angle]
            
    return False

def image_callback(data):

    global turn_state, start_turn_time

    if turn_state:
        print("In turn state")
        if (time.time() - start_turn_time > TIME_TURN):
            turn_state = False
        else:
            return

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, (20, 80, 120), (40, 255, 255))

    black_pub.publish(bridge.cv2_to_imgmsg(black, 'mono8'))

    #cv2.imshow("BW image", black)

    dir_state = {"top":     [False, []],
                 "right":   [False, []],
                 "left":    [False, []]
                 }

    # Check all halfs (left, right, top) about line

    img_top_half = black[:TOP_Y, TOP_X - 40:TOP_X + 40]
    #cv2.imshow("top half", img_top_half)
    dir_state["top"] = check_top(img_top_half, cv_image)

    img_r_half = black[:RIGHT_T_Y, RIGHT_T_X:]
    #cv2.imshow("right half", img_r_half)
    dir_state["right"] = check_right(img_r_half, cv_image)

    img_l_half = black[:LEFT_T_Y, :LEFT_T_X]
    #cv2.imshow("left half", img_l_half)
    dir_state["left"] = check_left(img_l_half, cv_image)

    print(dir_state)

    if dir_state["top"] and not dir_state["left"] and not dir_state["right"]: # only forward
        
        print("Going Forward")

        rect_top = dir_state["top"][1]

        rect_top[2] = convert_angle(angle=rect_top[2], w_min=rect_top[1][0], h_min=rect_top[1][0])

        center = cv_image.shape[1] / 2
        error = rect_top[0][0] - center

        #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

        cv2.circle(cv_image, (int(rect_top[0][0]), int(rect_top[0][1])), 5, (0, 0, 255), 3)

        #print(round(angle, 2), error)
        set_velocity(vx=SPEED_X, vy=error*(-0.005), vz=0, yaw=float('nan'), yaw_rate=rect_top[2]*(-0.008), frame_id='body')

    elif dir_state["top"] and not dir_state["left"] and dir_state["right"]: # waiting for right T cross
        
        rect_top = dir_state["top"][1]
        rect_right = dir_state["right"][1]

        rect_top[2] = convert_angle(angle=rect_top[2], w_min=rect_top[1][0], h_min=rect_top[1][1])

        if rect_right[0][1] < HEIGHT // 2:
            print("Going to Right T Cross")
            center = cv_image.shape[1] / 2
            error = rect_top[0][0] - center

            #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

            cv2.circle(cv_image, (int(rect_top[0][0]), int(rect_top[0][1])), 5, (0, 0, 255), 3)

            #print(round(angle, 2), error)
            set_velocity(vx=SPEED_X, vy=error*(-0.005), vz=0, yaw=float('nan'), yaw_rate=rect_top[2]*(-0.008), frame_id='body')
        else:
            print("Turning Right")
            #navigate(x=0, y=0, z=0, frame_id="body", yaw=-1.57, yaw_rate=0, auto_arm=False)
            #rospy.sleep(1)
            telem = get_telemetry()
            navigate(x=0, y=0, z=0, frame_id="body", yaw=-1.6, yaw_rate=0, auto_arm=False)
            rospy.sleep(2)
            print("rotated")
            navigate(x=telem.x, y=telem.y, z=FLY_HEIGHT, frame_id="aruco_map", yaw=telem.yaw - 1.6, yaw_rate=0, auto_arm=False)
            rospy.sleep(2)
            print("upped")
            navigate(x=1, y=0, z=0, frame_id="body", yaw=0, yaw_rate=0, auto_arm=False)
            rospy.sleep(6)
            print("translated")
            turn_state = True
            start_turn_time = time.time()

    elif dir_state["top"] and dir_state["left"] and not dir_state["right"]: # waiting for right T cross
        
        rect_top = dir_state["top"][1]
        rect_left = dir_state["left"][1]

        rect_top[2] = convert_angle(angle=rect_top[2], w_min=rect_top[1][0], h_min=rect_top[1][1])

        if rect_left[0][1] < HEIGHT // 2:
            print("Going to Left T Cross")
            center = cv_image.shape[1] / 2
            error = rect_top[0][0] - center

            #print(f"{x_min}, {y_min}, {w_min}, {h_min}, {round(angle, 2)}, {error}")

            cv2.circle(cv_image, (int(rect_top[0][0]), int(rect_top[0][1])), 5, (0, 0, 255), 3)

            #print(round(angle, 2), error)
            set_velocity(vx=SPEED_X, vy=error*(-0.005), vz=0, yaw=float('nan'), yaw_rate=rect_top[2]*(-0.008), frame_id='body')
        else:
            print("Turning Left")
            #navigate(x=0, y=0, z=0, frame_id="body", yaw=-1.57, yaw_rate=0, auto_arm=False)
            #rospy.sleep(1)
            telem = get_telemetry()
            navigate(x=0, y=0, z=0, frame_id="body", yaw=1.57, yaw_rate=0, auto_arm=False)
            rospy.sleep(2)
            print("rotated")
            navigate(x=telem.x, y=telem.y, z=FLY_HEIGHT, frame_id="aruco_map", yaw=telem.yaw + 1.57, yaw_rate=0, auto_arm=False)
            rospy.sleep(2)
            print("upped")
            navigate(x=1, y=0, z=0, frame_id="body", yaw=0, yaw_rate=0, auto_arm=False)
            rospy.sleep(6)
            print("translated")
            turn_state = True
            start_turn_time = time.time()
    else:
        print("Staying")

        navigate(x=0, y=0, z=0, frame_id="body", yaw=0, yaw_rate=0, auto_arm=False)

    debug.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    #cv2.imshow("Original image", cv_image)

    #cv2.waitKey(1)

print("Taking off")
navigate(x=0, y=0, z=FLY_HEIGHT, frame_id="body", speed=0.5, auto_arm=True)
rospy.sleep(5) 

print("Navigating to start point")
navigate_wait(x=0.1, y=3.5, z=FLY_HEIGHT, yaw=0, frame_id="aruco_map", auto_arm=False)
print("Navigated")
rospy.sleep(5)

print("Starting line")

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

rospy.spin()