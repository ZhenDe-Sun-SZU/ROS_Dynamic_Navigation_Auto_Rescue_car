#!/usr/bin/env python
# coding=utf-8
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist
from wheeltec_robot_rc.msg import pointss
import sys, select, termios, tty

a = 0.0
b = 0.0

speed = 0.04   #默认移动速度 m/s
turn  = 0.05   #默认转向速度 rad/s

def callback(data):
    rospy.loginfo("Received x1: %s, y1: %s, x2: %s, y2: %s,",data.x1,data.y1,data.x2,data.y2)
    a=abs(data.x1-data.x2)
    b=abs((data.x1+data.x2)/2)
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5) #创建速度话题发布者，'~cmd_vel'='节点名/cmd_vel'
    rospy.loginfo("Received b: %s",b)
    twist = Twist() #创建ROS速度话题变量
    if 0<a<102.0 and 265.0<=b<365.0:
        twist.linear.x  = speed;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0;  twist.angular.y = 0;  twist.angular.z = 0
   
    elif 0<a<102.0 and 0<b<265.0:
        twist.linear.x  = speed;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0;  twist.angular.y = 0;  twist.angular.z = turn
                #rospy.loginfo("Received")

    elif 0<a<102.0 and 365.0<=b:
        twist.linear.x  = speed;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0;  twist.angular.y = 0;  twist.angular.z = -turn

    elif a>102.0:
        twist.linear.x  = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0;  twist.angular.y = 0;  twist.angular.z = 0

    elif a==0.0:
        twist.linear.x  = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0;  twist.angular.y = 0;  twist.angular.z = 0 
                   
    pub.publish(twist) #ROS发布速度话题    


#主函数
if __name__=="__main__":

    rospy.init_node('point_listener',anonymous=True)
    rospy.Subscriber("points_chatter",pointss,callback)
    #rospy.spin()
    
            
    rospy.spin()
