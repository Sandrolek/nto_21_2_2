import rospy
import numpy as np
import time

import cv2
from cv_bridge import CvBridge

from clover import srv
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool
from clover.srv import SetLEDEffect
from math import *

rospy.init_node('main')


######################
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point32, PointStamped

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

######################

bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

res_pub = rospy.Publisher('res', Image, queue_size=1)

red_pub = rospy.Publisher('mask/red', Image, queue_size=1)
yellow_pub = rospy.Publisher('mask/yellow', Image, queue_size=1)
blue_pub = rospy.Publisher('mask/blue', Image, queue_size=1)
green_pub = rospy.Publisher('mask/green', Image, queue_size=1)

masks = {
        'red': [((0, 100, 155), (10, 255, 255)), red_pub],
        'yellow': [((20, 80, 120), (40, 255, 255)), yellow_pub],
        'blue': [((110, 70, 115), (130, 255, 255)), blue_pub],
        'green': [((50, 130, 200), (70, 255, 255)), green_pub]
        }

relatives = {
    0: 'circle',
    1: 'square'
}

objects = {
    "red circle": list(),
    "blue circle": list(),
    "yellow square": list(),
    "green square": list()
}

FLY_HEIGHT = 1.3

points_3D = np.array([(-0.01,0.01,0.0), (0.01,0.01,0.0), (0.01,-0.01,0.0), (-0.01,-0.01,0.0)])

def navigate_wait(x=0, y=0, z=0, yaw=radians(90), speed=0.3, frame_id='', auto_arm=False, tolerance=0.15, func=None):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        if func:
            func()
        telem = get_telemetry(frame_id='navigate_target')
        if sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.1)

def count_dist(x0, y0, x1, y1):

    return sqrt((x1-x0)**2 + (y1-y0)**2)

def convert_coords(x0, y0):
    global camera_matrix, dist_coeffs, res_pose
    
    points_2D = np.array([[x0-2, y0-2], [x0-2, y0+2], [x0+2, y0+2], [x0+2, y0-2]], dtype="float")

    success, rotation_vector, translation_vector = cv2.solvePnP( 
        points_3D, points_2D, camera_matrix, dist_coeffs, flags=0 )

    # line_point, jacobian = cv2.projectPoints( np.array([(0.0,0.0,-0.5)]),
    #     rotation_vector, translation_vector, camera_matrix, dist_coeffs )

    # for p in points_2D:
    #     cv2.circle(cv_image, (int(p[0]), int(p[1])), 3, (0,0,255), -1)

    # point1 = (int(points_2D[0][0]), int(points_2D[0][1]))
    # point2 = (int(line_point[0][0][0]), int(line_point[0][0][1]))
    # cv2.line(cv_image, point1, point2, (255,255,255), 2)

    # pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    #print()
    
    pose = PoseStamped()
    pose.header.frame_id = 'main_camera_optical'
    pose.header.stamp = rospy.get_rostime()
    
    pose.pose.position = Point32(*translation_vector)
    # print(f"Translation vector: {translation_vector}")
    # print(f"Rotation_vector: {rotation_vector}")

    frame_id = 'aruco_map' 
    transform_timeout = rospy.Duration(0.2)

    new_pose = tf_buffer.transform(pose, frame_id, transform_timeout)

    res_x = new_pose.pose.position.x
    res_y = new_pose.pose.position.y

    return (res_x, res_y)

def find_objects_cb(data):

    samples = np.loadtxt('generalsamples.data', np.float32)
    responses = np.loadtxt('generalresponses.data', np.float32)
    responses = responses.reshape((responses.size, 1))

    model = cv2.ml.KNearest_create()
    model.train(samples, cv2.ml.ROW_SAMPLE, responses)

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    out = cv_image
    
    for mask in masks:

        cur = cv2.inRange(hsv, masks[mask][0][0], masks[mask][0][1])

        out = cv_image

        #ret, thresh = cv.threshold(gray, 70, 255, cv.THRESH_BINARY_INV)

        contours, hierarchy = cv2.findContours(cur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]      

        contours = [i for i in contours if cv2.contourArea(i) > 1000]

        for cnt in contours:
            [x, y, w, h] = cv2.boundingRect(cnt)
            center = (x + w // 2, y + h // 2)
            try:
                #print("is contour")
                
                roi = cur[y:y + h, x:x + w]
                roismall = cv2.resize(roi, (10, 10))
                roismall = roismall.reshape((1, 100))
                roismall = np.float32(roismall)
                
                retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
                
                num = int(results[0][0]) # ?
                
                #print(relatives[num])
                real_coords = [round(i, 2) for i in list(convert_coords(*center))]
                res_str = str(f"{mask} {relatives[num]} {real_coords}")

                if str(mask + ' ' + relatives[num]) not in objects.keys():
                    continue

                arr = objects[str(mask + ' ' + relatives[num])]

                for point in arr:
                    if count_dist(real_coords[0], real_coords[1], point[0], point[1]) < 0.5:
                        break
                else:
                    arr.append(real_coords)
                
                cv2.putText(out, res_str, (x + w // 2, y + h // 2), 0, 0.25, (0, 0, 255))
                if relatives[num] == 'circle':
                    cv2.circle(out, center, w // 2, (0, 0, 255), 2)
                else:
                    cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
                
            except cv2.Error as e:
                print('Invalid')

        pub = masks[mask][1]
        res_pub.publish(bridge.cv2_to_imgmsg(out, 'bgr8'))

#####################################

camera_matrix = np.zeros((3,3))
dist_coeffs = np.array([])

def convert_cam_info(msg):
    global camera_matrix, dist_coeffs
    dist_coeffs = np.array(msg.D)

    for i in range(3):
        for j in range(3):
            camera_matrix[i][j] = msg.K[i*3+j]

cam_info = rospy.wait_for_message('/main_camera/camera_info', CameraInfo)
convert_cam_info(cam_info)
 
#####################################

start_time = time.time()
print("Started")
#navigate_wait(x=0, y=0, z=1, speed=0.2, frame_id='body', auto_arm=True, func=find_objects)
navigate_wait(x=0, y=0, z=FLY_HEIGHT, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(4)

navigate_wait(x=0, y=0, z=FLY_HEIGHT, speed=0.2, frame_id='aruco_map')

print("Got to started point")

points = [
    (1.5, 4, FLY_HEIGHT),
    (3, 1, FLY_HEIGHT),
    (3.5, 3, FLY_HEIGHT)
]

img_sub = rospy.Subscriber('/main_camera/image_raw_throttled', Image, find_objects_cb)

for x0, y0, z0 in points:
    print(f"Navigating to ({x0}, {y0})")
    navigate_wait(x=x0, y=y0, z=z0, speed=0.1, frame_id="aruco_map")

img_sub.unregister()

land()

print(objects)

fly_time = time.time() - start_time
#fly_time_str = str(int(fly_time // 60)) + ' min ' + str(int(fly_time % 60)) + ' sec'

print(f"Flight took {fly_time} seconds")