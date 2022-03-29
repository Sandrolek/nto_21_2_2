import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

from clover import srv
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool
from clover.srv import SetLEDEffect
import math

import numpy as np
import cv2

rospy.init_node('main')

# TRANSLATION
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point32, PointStamped

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
#

bridge = CvBridge()

pub = rospy.Publisher('res', Image, queue_size=1)

points_3D = np.array([(-0.01,0.01,0.0), (0.01,0.01,0.0), (0.01,-0.01,0.0), (-0.01,-0.01,0.0)])
res_pose = None

def navigate_wait(x=0, y=0, z=0, yaw=math.radians(90), speed=0.3, frame_id='', auto_arm=False, tolerance=0.15):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(10)

def image_cb(data):
    global camera_matrix, dist_coeffs, res_pose

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = pyzbar.decode(cv_image)
    
    '''
    pose = PoseStamped()
    pose.header.frame_id = 'main_camera_optical'
    pose.header.stamp = rospy.get_rostime()
    
    pose.pose.position = Point32(*translation_vector)
    print(f"Translation vector: {translation_vector}")
    print(f"Rotation_vector: {rotation_vector}")
    
    pose.pose.position.x = x0
    pose.pose.position.y = y0
    pose.pose.position.z = 0
    pose.pose.orientation.w = 1.0

    frame_id = 'aruco_map'
    transform_timeout = rospy.Duration(0.2)

    new_pose = tf_buffer.transform(pose, frame_id, transform_timeout)
    '''  
    
    for barcode in barcodes:
        (left, top, w, h) = barcode.rect

        x0 = left + w / 2
        y0 = top + h / 2
        
        #polygon = barcode.polygon
        points_2D = np.array([[x0-2, y0-2], [x0-2, y0+2], [x0+2, y0+2], [x0+2, y0-2]], dtype="float")
        #points_2D = np.array([i for i in polygon], dtype="float")
        #points_2D = np.array([[x0-2, y0-2], [x0-2, y0+2], [x0+2, y0+2], [x0+2, y0-2]], dtype="float")

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

        #print()

        # cv2.rectangle(cv_image, (left, top), (left + w, top + h), (255, 0, 0), 2)
        pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

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

        print("New pose: ")
        print(f"{new_pose.pose.position.x}, {new_pose.pose.position.y}")

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

image_sub = rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_cb)

print("Started")
navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)
print("Got off")

navigate_wait(x=1.5, y=1.5, z=1, frame_id='aruco_map')
print("Got to Started point")

while True:
    rospy.sleep(0.2)

