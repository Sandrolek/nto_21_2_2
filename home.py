def home(xy):
    z = 2
    s = list(map(float, xy)) #xy - твоя переменная вместо xy
    navigate_wait(x = s[0], y= s[1], z = 2, frame_id='aruco_map')
    rospy.sleep(3)
    while rospy.wait_for_message('rangefinder/range', Range).range > 0.3:
        navigate_wait(z = z, frame_id='body', speed = 1)
        print('land')
        z -= 0.2 
    if rospy.wait_for_message('rangefinder/range', Range).range <= 0.3:
        print('disarm')
        arming(False)
        