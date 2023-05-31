#! /usr/bin/env python

from cmath import sqrt
import rospy
import actionlib
import assignment_2_2022.msg
from assignment_2_2022.action import Positions, PositionsRequest, PositionsFeedback, PositionsResult
from assignment_2_2022.srv import check_status, check_statusResponse
from assignment_2_2022.msg import reaching_goal, status
import time
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
import actionlib
import actionlib.msg
from std_srvs.srv import *

#count number of goals reached and cancelled 
global counter_cancelled 
global counter_success
counter_cancelled = 0 #initialize 
counter_success = 0 #initialize




## Subscriber to /reaching_goal/status
reaching_goal= "/reaching_goal/status"



## name service 
node_check_status = "/check_status"



def status_callback(msg):
    global counter_cancelled 
    global counter_success
    for goal_status in msg.status_list:
        if goal_status.status == GoalStatus.ACTIVE: # The goal is currently being processed by the action server
            
            rospy.loginfo("Goal is active")
        elif goal_status.status == GoalStatus.SUCCEEDED: # The goal was achieved successfully by the action server (Terminal State)
            rospy.loginfo("Goal succeeded")
            counter_success+=1
            print('number of goals success is %s', str(counter_success))
        elif goal_status.status == GoalStatus.ABORTED: # The goal was aborted during execution by the action server due to some failure (Terminal State)
            rospy.loginfo("Goal aborted")
        elif goal_status.status == GoalStatus.PREEMPTED: # The goal received a cancel request after it started executing and has not yet completed execution
            rospy.loginfo("Goal preempted")
            counter_cancelled+=1
            print('number of goals cancelled is %s', str(counter_cancelled))
        elif goal_status.status == GoalStatus.REJECTED:  #The goal was rejected by the action server without being processed,because the goal was unattainable or invalid (Terminal State)
            rospy.loginfo("Goal rejected")
        elif goal_status.status == GoalStatus.RECALLED: # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
            rospy.loginfo("Goal recalled")
    
#define the service callback function that will be called when the service is requested
def check_status_srv(req):
    global counter_cancelled 
    global counter_success
    res = check_statusResponse()
    res.counter_cancelled_srv = counter_cancelled
    res.counter_success_srv = counter_success
    return res



def main(args=None):
     
        
       #get information from /reaching_goal/status 
       status_sub = rospy.Subscriber(reaching_goal, GoalStatusArray, status_callback)
       
       #name of service "check_status"
       service = rospy.Service('check_status', check_status, check_status_srv)

       rate = rospy.Rate(20)
       while not rospy.is_shutdown():
            rate.sleep()

       rospy.spin()
         

if __name__ == "__main__":
    # Init ROS1 and give the node a name 
    rospy.init_node("check_status")
    main()
        
        
    
