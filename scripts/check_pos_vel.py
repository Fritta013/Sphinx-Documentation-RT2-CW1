#! /usr/bin/env python

from cmath import sqrt
import rospy
import actionlib
import assignment_2_2022.msg
from assignment_2_2022.action import Positions, PositionsRequest, PositionsFeedback, PositionsResult, PositionsGoal
from assignment_2_2022.msg import RobotPose
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
from std_srvs.srv import *

rate_input=10

## name service 
node_check_pos_vel = "/check_pos_vel"


global x
global y
global goal_x
global goal_y


def message_callback(data):
    global x
    global y
    x = data.current_x
    y = data.current_y
    print("current x is: ", x)
    print("current y is: ", y)



def action_callback(data):
    global goal_x
    global goal_y
    goal_x = data.target_x
    goal_y = data.target_y
    print("target x is set at: ", goal_x)
    print("target y is set at: ", goal_y)




def main(args=None):

    

       
       #subscribe to custom message RobotPose to get current positions x and y publised by "UI" node
       sub_current_xy = rospy.Subscriber("robot_pose", RobotPose, message_callback)
    
       #subscribe to action get target x and y
       sub_target_xy = rospy.Subscriber('reaching_goal',PositionsRequest,action_callback)
       

       #compute distance from the obtained goal x, y and from the taget x,y
       distance = sqrt( ( (goal_x - x)**2) + ( (goal_y - y)**2) )
       rospy.loginfo("distance from goal %s ", str(distance) )
       print("distance from goal %s ", str(distance) )
       rospy.spin()
       
       #parameter to set how fast the node publishes the information
       rate = rospy.Rate(rate_input)
       while not rospy.is_shutdown():
            rate.sleep()
    

    

if __name__ == '__main__':
    # Init ROS1 and give the node a name
    rospy.init_node("check_pos_vel")
    main()



