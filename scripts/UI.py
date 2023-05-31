#! /usr/bin/env python

import rospy
import actionlib
import assignment_2_2022.msg
import time
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2022.msg import RobotPose


linear_x = 0
linear_y = 0
 
def odom_callback(data):
    global linear_x, linear_y
    linear_x = data.twist.linear.x
    linear_y = data.twist.linear.y



class UpdatePositionsClient():
    def __init__(self):
       # Initializes _client node
       self._action_client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2022.msg.PositionsAction)
       self._publisher = rospy.Publisher('robot_pose', RobotPose, queue_size=10)

    # Waits for server to be available, then sends goal
    def send_goal(self, x , y):
        goal_msg = assignment_2_2022.msg.PositionsGoal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        rospy.loginfo('Waiting for server...')

        self._action_client.wait_for_server()

        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self._send_goal_future = self._action_client.send_goal(goal_msg, active_cb=self.goal_response_callback, feedback_cb=self.feedback_callback, done_cb = self.get_result_callback)

        rospy.loginfo("Goal sent!")
        
    # Run when client accepts goal
    def goal_response_callback(self):
        rospy.loginfo('Goal accepted :)')

   # Run when action server sends feedback
    def feedback_callback(self, feedback_msg):
        rospy.loginfo('Received feedback x at: {0}'.format(feedback_msg.current_x))
        rospy.loginfo('Received feedback y at: {0}'.format(feedback_msg.current_y))
        

        # Create custom message
        msg = RobotPose()
        # Populate message fields with current position and velocity data
        msg.current_x = feedback_msg.current_x
        msg.current_y = feedback_msg.current_y 
        msg.current_vx = linear_x
        msg.current_vy = linear_y
        # Publish message on topic
        self._publisher.publish(msg)
        rospy.loginfo('Published robot position and velocity: x={0}, y={1}, vx={2}, vy={3}'.format(feedback_msg.current_x, feedback_msg.current_y, linear_x, linear_y))
    
    
    # Run when client sends final result
    def get_result_callback(self, state, result):
        # Show log and exit node
        if result.is_finished:
            rospy.loginfo('Result: {0}'.format(result.is_finished))
        else:
            rospy.loginfo("Goal failed")

    #method to allow the user to cancel target positions x and y that was sent to the server
    def cancel_goal(self):
        self._action_client.cancel_goal()


    
        
def main(args=None):
# Init ROS1 and give the node a name

    
    while not rospy.is_shutdown():

        #subscribe to the /odom topic using a subscriber
        odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)   

        # taking the target positions from user 
        try:
            x, y = float(input("Enter target positions x and y, seperated by a white space: ").split())
            print("Target x is {} and Target y is {}".format(x,y))
            print()
        except ValueError:#in case the user enters wrong values 
            print("Invalid input. Please enter two numeric values.")
            continue
        action_client = UpdatePositionsClient()
        # Sends goal and waits until it's completed
        action_client.send_goal(x,y) 


        # Check if user wants to cancel the goal
        cancel_input = input("if you want to cancel then type 'cancel', or type any other key if you want to continue: ")
        if cancel_input == "cancel": #user typed cancel 
                action_client.cancel_goal()
                print("Your Goal got cancelled!")
                continue #essential!! to allow the program to go back to the beginning of the loop and ask for new targets
        
        rospy.spin()

if __name__ == '__main__':
   rospy.init_node("UI")
   main()
