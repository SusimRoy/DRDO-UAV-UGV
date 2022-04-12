#!/usr/bin/python
import rospy
from prius_msgs.msg import Control
from geometry_msgs.msg import Twist


def velocity_callback(msg):
    vel_msg.throttle = 0.3/10

    vel_msg.steer=msg.angular.z*10

rospy.init_node('prius', anonymous=True)
velocity_subscriber = rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, velocity_callback)
velocity_publisher = rospy.Publisher('/prius', Control, queue_size=10)
rate=rospy.Rate(2)
vel_msg = Control()

while not rospy.is_shutdown():
    velocity_publisher.publish(vel_msg)
    rate.sleep()
