#!/usr/bin/env python
from pickle import TRUE
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
import numpy as np
x=0
y=0
z=0
yaw=0

TIMER = 0
TIME_STEP = 0.001
KP = 0.1
KI = 0.01
KD = 0.01
Kp_Angle=1.4
Kd_Angle=5
Ki_Angle=0
Derivator=0
Integrator=0
Derivator1=0
Integrator1=0
I_max=50
I_min=-50

def poseCallback(pose_message):
    global x,y,z,yaw
    x=pose_message.x
    y=pose_message.y
    yaw=pose_message.theta



def compute_angle(error):
    global Integrator, Derivator, yaw

    #error = desired_angle_goal-yaw

    P_value = Kp_Angle * error
    D_value = Kd_Angle * (error-Derivator)
    Derivator = error
    Integrator += error
    if Integrator > I_max:
	    Integrator = I_max
    elif Integrator < I_min:
	    Integrator = I_min
    I_value = Integrator * Ki_Angle
    angular_speed = P_value + I_value + D_value
    return angular_speed

def compute_distance(error):
    global Integrator1, Derivator1, yaw

    P_value = KP * error
    D_value = KD * (error-Derivator)
    Derivator1 = error
    Integrator1 += error
    if Integrator1 > I_max:
	    Integrator1 = I_max
    elif Integrator < I_min:
	    Integrator1 = I_min
    I_value = Integrator1 * KI
    linear_speed = P_value + I_value + D_value
    return linear_speed    

def rotate(x_goal,y_goal):
    global x,y,z,yaw
    #print(yaw)
    velocity_message=Twist()
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
    #print(yaw)
    while TRUE:
        distance=abs(math.sqrt(((x-x_goal)**2)+(y-y_goal)**2))
        desired_angle_goal=math.atan2((y_goal-y),(x_goal-x))
        clockwise=0
        diff_an=desired_angle_goal-yaw
        angular_speed=compute_angle(desired_angle_goal-yaw)
        if desired_angle_goal-yaw <0:
            clockwise=1
        if(clockwise ==1):
            velocity_message.angular.z=-abs(angular_speed)
            print(" velocity {}".format(velocity_message.angular.z))
        else:
            velocity_message.angular.z=abs(angular_speed)   
 
        velocity_publisher.publish(velocity_message)

        if abs(diff_an)<0.005:
            break



def move(x_goal,y_goal):
    global x,y,z,yaw
    #print(yaw)
    velocity_message=Twist()
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
    #print(yaw)
    while TRUE:
        distance=abs(math.sqrt(((x-x_goal)**2)+(y-y_goal)**2))
        clockwise=0

        linear_speed=compute_distance(distance)
 
        velocity_message.linear.x=linear_speed
        
        
        velocity_publisher.publish(velocity_message)

        print("x= {} ,y={} ".format(x,y))
        print("distance {}".format(distance))

        if(distance<0.1):
            break

if __name__=='__main__':
    try:
        rospy.wait_for_service('turtle1/teleport_absolute')
        rospy.wait_for_service('turtle1/teleport_absolute')
        rospy.init_node("turtlesim_motion_pose",anonymous=True)
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
        position_topic="/turtle1/pose"
        pose_subscriber=rospy.Subscriber(position_topic,Pose,poseCallback)
        rate = rospy.Rate(10)
        
        time.sleep(2)
        fp = open('nodes/points.txt', 'r')

        points = []
        for line in fp:
            if(line=='\n'):
                break
            point = line.split()
            x1=(float(point[1]))/60.0
            y1=11-((float(point[0])))/60.0
            points.append((x1,y1))
            #velocity_publisher.publish()
        for id in range(len(points)):
            rotate(points[id][0],points[id][1])
            move(points[id][0],points[id][1])


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")  
 
