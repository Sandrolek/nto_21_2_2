import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

import numpy as np
import cv2

rospy.init_node('main')

################
# TRANSLATION #
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point32

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
################

bridge = CvBridge()

pub = rospy.Publisher('res', Image, queue_size=1)

points_3D = np.array([(-0.23,0.23,0.0), (0.23,0.23,0.0), (0.23,-0.23,0.0), (-0.23,-0.23,0.0)])
res_pose = None

def image_cb(data):
	global camera_matrix, dist_coeffs, res_pose

	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	barcodes = pyzbar.decode(cv_image)

	for barcode in barcodes:
			
		# (left, top, w, h) = barcode.rect

		# x0 = left + w / 2
		# y0 = top + h / 2
		polygon = barcode.polygon
		points_2D = np.array([i for i in polygon], dtype="float")

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
		pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

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
		print("New pose: ", end='')
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

while True:
	rospy.sleep(0.2)

