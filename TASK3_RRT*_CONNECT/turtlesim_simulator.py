#!/usr/bin/env python
from pickle import TRUE
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import numpy as np
x=0
y=0
z=0
yaw=0

TIMER = 0
TIME_STEP = 0.001
KP = 0.36
KI = 40.0
KD = 0.0008099999999999997

def poseCallback(pose_message):
    global x,y,z,yaw
    x=pose_message.x
    y=pose_message.y
    yaw=pose_message.theta



def move(speed,goal,is_forward):

    velocity_message=Twist()
    global x,y
    x0=x
    y0=y
    epsilon=4	
    if(is_forward):
        velocity_message.linear.x=abs(speed)
    else:
        velocity_message.linear.x=-abs(speed)
    distance_moved=0.0
    loop_rate=rospy.Rate(10)
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
    while True:
        rospy.loginfo("TURTLESIM MOVES BACKWARD")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved+=abs(0.5*math.sqrt((x-x0)**2+(y-y0)**2))
        print(distance_moved)
        if not abs(0.5*math.sqrt((x-goal[0])**2+(y-goal[1])**2))<epsilon):
            rospy.loginfo("reached")
            break

    velocity_message.linear.x=0
    velocity_publisher.publish(velocity_message)

def rotate(angular_speed_degree,relative_angle_degree,clockwise):
    global yaw
    velocity_message=Twist()
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0

    thetat0=yaw
    angular_speed=math.radians(abs(angular_speed_degree))
    if(clockwise):
        velocity_message.angular.z=-abs(angular_speed)
    else:
        velocity_message.angular.z=abs(angular_speed)
        velocity_message.angular.z=abs(angular_speed)


    angle_moved=0.0
    loop_rate=rospy.Rate(10)
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

    t0=rospy.Time.now().to_sec()
    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)
        t1=rospy.Time.now().to_sec()
        current_angle_degree=(t1-t0)*angular_speed_degree
        loop_rate.sleep()
        if(current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break
    velocity_message.angular.z=0
    velocity_publisher.publish(velocity_message)

def go_to_goal(x_goal,y_goal):
    global x,y,z,yaw
    velocity_message=Twist()
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
    while TRUE:
        K_linear=0.5
        distance=abs(math.sqrt(((x-x_goal)**2)+(y-y_goal)**2))
        linear_speed=distance*K_linear
        K_angular=4.0
        desired_angle_goal=math.atan2(y_goal-y,x_goal-x)
        angular_speed=(desired_angle_goal-yaw)*K_angular
        velocity_message.linear.x=linear_speed
        velocity_message.angular.z=angular_speed
        velocity_publisher.publish(velocity_message)
        print("x= {} ,y={} ".format(x,y))

        if(distance<0.01):
            break


def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians=desired_angle_radians-yaw
    if relative_angle_radians<0:
        clockwise=1
    else :
        clockwise=0
    print(relative_angle_radians,desired_angle_radians)   
    rotate(30,math.degrees(abs(relative_angle_radians),clockwise)) 
class PID(object):
	def __init__(self,KP,KI,KD,target):
		self.kp = KP
		self.ki = KI
		self.kd = KD 
		self.setpoint = target
		self.error = 0
		self.integral_error = 0
		self.error_last = 0
		self.derivative_error = 0
		self.output = 0
	def compute(self, pos):
		self.error = self.setpoint - pos
		#self.integral_error += self.error * TIME_STEP
		self.derivative_error = (self.error - self.error_last) / TIME_STEP
		self.error_last = self.error
		self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error
		if((((self.error>=0) and (self.integral_error>=0))or((self.error<0) and (self.integral_error<0)))):
				self.integral_error += self.error * TIME_STEP
		else:
			#rectangular integration
			self.integral_error += self.error * TIME_STEP

		return self.output
		
	def get_kpe(self):
		return self.kp*self.error
	def get_kde(self):
		return self.kd*self.derivative_error
	def get_kie(self):
		return self.ki*self.integral_error

if __name__=='__main__':
    try:
        rospy.init_node("turtlesim_motion_pose",anonymous=True)
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
        position_topic="/turtle1/pose"
        pose_subscriber=rospy.Subscriber(position_topic,Pose,poseCallback)
        time.sleep(2)
        paths=[(574, 541), (548, 516), (520, 493), (491, 472), (507, 440), (491, 408), (475, 376), 
        (484, 342), (462, 314), (467, 279), (459, 244), (424, 241), (435, 241), (424, 241), (435, 241), (415, 213),
         (382, 202), (365, 172), (335, 156), (312, 182), (287, 159), (267, 131), (233, 132), (199, 131), (165, 132), (145, 104), 
         (114, 90), (82, 76), (57, 52)]
	
        
        for path in paths:
            move(1.0,(path[0],path[1]),False)
            setDesiredOrientation(math.radians(90))

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
