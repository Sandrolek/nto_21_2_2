from mavros_msgs.srv import CommandBool

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

def home():    
    s = list(map(float, xy)) 
    navigate_wait(x = s[0], y= s[1], z = 2)
    rospy.sleep(3)
    while rospy.wait_for_message('rangefinder/range', Range).range > 0.3:
        navigate_wait(frame_id='body', z = -0.2, speed = 1)
        print('land')         
    if rospy.wait_for_message('rangefinder/range', Range).range <= 0.3:
        print('disarm')
        arming(False)
xy = [2, 2]       
home()