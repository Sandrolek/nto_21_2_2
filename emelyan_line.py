import cv2 as cv
import numpy as np
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('main')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def navigate_line(imm, im, v):
    a, b = center(imm, im) #определение изменение маршрута
    a = math.atan(a/80) #определяю угол на который нужно повернуть
    if b == 0:
        navigate_wait(x=0.1, y=0, z=0, yaw=0, speed=0.5, frame_id='body', auto_arm=False)
    else:
        navigate_wait(x=0, y=0, z=0, yaw=a*b, speed=0.5, frame_id='body', auto_arm=False)
    #v.append([get_telemetry.x, get_telemetry.y]) #записываю корды, чтобы в дальшейем не повторятся

def center(imm,im):
    '''imm[118:120, 149:169] = [255, 128, 0]
    imm[109:129, 158:160] = [255, 128, 0]
    cv.circle(imm, (239, 119), 10, (0, 0, 255), 1)
    imm[118:120, 230:250] = [255, 200, 0]
    imm[109:129, 238:240] = [255, 200, 0]'''
    #здесь я определяю отклонение поворота в пикселях
    if np.all(im[118:120, 238:240] == [0, 0, 0]):
        for i in range(240):
            if np.all(im[i, 240] == [255, 255, 255]):
                imm[i, 238:240] = [0, 0, 255]
                if i < 119:
                    return 120-i, -1
                else:
                    return i-120, 1
    else:
        return 0, 0

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
'''im = cv.imread("C:/Users/EEI/Desktop/saerjtyk.png")
imm = cv.imread("C:/Users/EEI/Desktop/saerjtyk.png")
imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 127, 255, 0)
contours, img = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
print(contours)
center(imm,im)'''
#cv.drawContours(imm, contours, -1, (0,255,0), 3)
'''cv.imshow('contours', imm)
cv.waitKey(0)
cv.destroyAllWindows()'''

k = 0
sob = True
navigate_wait(x=0, y=0, z=1, yaw=0, speed=0.5, frame_id='body', auto_arm=True)
#navigate_wait(x=0, y=0.5, z=0, yaw=0, speed=0.5, frame_id='body', auto_arm=True)

v = []
while sob:
    im = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    im = cv.cvtColor(im, cv.COLOR_BGR2HSV)
    im = cv.inRange(im, np.array([50, 202, 59]), np.array([71, 255, 255]))
    navigate_line(im,im, v)
    '''
    imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(imgray, 127, 255, 0)
    contours, img = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cv.imshow('contours', imm)
    '''
    k+=1
    if k == 100:
        sob = False #ограничение исполнения


'''
def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    # Do any image processing with cv2...

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
'''