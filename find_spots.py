import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import numpy as np
import math

import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('find_spots')

################
# TRANSLATION #
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point32

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
################

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


bridge = CvBridge()

HEIGHT = 240
WIDTH = 320

camera_matrix = np.zeros((3,3))
dist_coeffs = np.array([])

# for solvepnp
#points_3D = np.array([(-0.23,0.23,0.0), (0.23,0.23,0.0), (0.23,-0.23,0.0), (-0.23,-0.23,0.0)])
points_3D = np.array([(-0.23,0.115,0.0), (0.23,0.115,0.0), (0.23,-0.115,0.0), (-0.23,-0.115,0.0)])

mask = ((2, 85, 168), (12, 255, 255))

k = 0.25 / 2300

def convert_cam_info(msg):
	global camera_matrix, dist_coeffs

	dist_coeffs = np.array(msg.D)

	for i in range(3):
		for j in range(3):
			camera_matrix[i][j] = msg.K[i*3+j]

def count_coords(rect=[], cv_image=None):

    global camera_matrix, dist_coeffs

    points_2D = np.array([i for i in rect], dtype="float")

    success, rotation_vector, translation_vector = cv2.solvePnP( 
        points_3D, points_2D, camera_matrix, dist_coeffs, flags=0 )

    #print(f"points 2D: {points_2D}")

    line_point, jacobian = cv2.projectPoints( np.array([(0.0,0.0,-0.5)]),
        rotation_vector, translation_vector, camera_matrix, dist_coeffs )

    for p in points_2D:
        cv2.circle(cv_image, (int(p[0]), int(p[1])), 3, (0,0,255), -1)

    point1 = (int(points_2D[0][0]), int(points_2D[0][1]))
    point2 = (int(line_point[0][0][0]), int(line_point[0][0][1]))
    cv2.line(cv_image, point1, point2, (255,255,255), 2)
        
    # cv2.rectangle(cv_image, (left, top), (left + w, top + h), (255, 0, 0), 2)
    # pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    pose = PoseStamped()
    pose.header.frame_id = 'main_camera_optical'  # фрейм, в котором задана позиция
    pose.header.stamp = rospy.get_rostime()  # момент времени, для которого задана позиция (текущее время)
    
    pose.pose.position = Point32(*translation_vector)
    #print(f"Translation vector: {translation_vector}")
    #print(f"Rotation_vector: {rotation_vector}")
    
    # pose.pose.position.x = x0
    # pose.pose.position.y = y0
    # pose.pose.position.z = 0
    # pose.pose.orientation.w = 1.0

    frame_id = 'aruco_map'  # целевой фрейм
    transform_timeout = rospy.Duration(0.2)  # таймаут ожидания трансформации

    new_pose = tf_buffer.transform(pose, frame_id, transform_timeout)

    res = (round(new_pose.pose.position.x, 2), round(new_pose.pose.position.y, 2))

    # print("New pose: ", end='')
    # print(f"{res[0]}, {res[1]}")

    return res

# def turn_point(x0, y0, l, r, angle):

#     x1=x*cos(angle)+y*sin(angle) 
#     y1=y*cos(angle)-x*sin(angle)

#     return (x1, y1)

def img_cb(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, mask[0], mask[1])
    cv2.imshow("BW image", black)

    cnts, _ = cv2.findContours(black.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) == 0:
        return

    cnts = list(filter(lambda x: True if cv2.contourArea(x) > 300 else False, cnts))

    res = []

    for cnt in cnts:
        
        # drawing rectangle
        
        rect = cv2.minAreaRect(cnt)
        (x_min, y_min), (w_min, h_min), angle = rect
        
        box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
        box = np.int0(box)
        cv2.drawContours(cv_image, [box], 0, (255, 0, 0),2)

        # counting Square
        cnt_area = cv2.contourArea(cnt)

        S = round(k * cnt_area, 2)


        # finding coords
        rect = [(x_min - w_min // 2, y_min - h_min // 2), (x_min + w_min // 2, y_min - h_min // 2),
                (x_min + w_min // 2, y_min + h_min // 2), (x_min - w_min // 2, y_min + h_min // 2)]

        global_coords = count_coords(rect=box, cv_image=cv_image)

        res.append(global_coords)
    
    print(res)

    cv2.imshow("Original image", cv_image)

    cv2.waitKey(1)

cam_info = rospy.wait_for_message('/main_camera/camera_info', CameraInfo)
convert_cam_info(cam_info)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, img_cb)

rospy.spin()