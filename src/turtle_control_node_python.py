#!/usr/bin/env python

import rospy                            # general include to proceed with ros and python
import sys                              # neccesary for sys command
from geometry_msgs.msg import Twist     # twist message
from turtlesim.msg import Pose          # pose message
from turtlesim.srv import Spawn         # spawn service from turtlesim


# callback function to be performed on each pose callback
def pose_callback_function(msg):
    rospy.loginfo(rospy.get_caller_id() + "received: %s", msg)




# main function to initialize the rosnode and perform turtle circle loop
def node(turtlename):
    # initialisierung des ros knotens
    rospy.init_node('turtle_control_node', anonymous=True)


    # generate the topics based on the turtles name
    turtle_twist_topic = turtlename + '/cmd_vel'
    turtle_pose_topic  = turtlename + '/pose'

    # Initialization of the publisher for the velocity
    twist_pub = rospy.Publisher('enter_pub_topic', Twist, queue_size=1)
    
    # Initialization of the subscriber
    pose_sub  = rospy.Subscriber('enter_sub_topic', Pose, pose_callback_function)


    rospy.wait_for_service('/spawn')
    try:
        x = 5.0
        y = 5.0 
        phi = 0.0
        spawn_turtle_service = rospy.ServiceProxy('enter_spawn_topic', Spawn)
        resp1 = spawn_turtle_service(x, y, phi, turtlename)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    
    # create loop to publish velocity command
    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
        ############################################
        #
        # ENTER CODE
        #
        ############################################
        twist_pub.publish(vel_msg)



        # wait till loop condition is true
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: my_node.py turtlename")
    else:
        node(sys.argv[1])