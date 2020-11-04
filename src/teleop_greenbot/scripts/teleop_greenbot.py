#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_greenbot')
import rospy
from geometry_msgs.msg import Twist

def callback(msg):

    rospy.loginfo(f"Linear: x = {msg.linear.x}, y = {msg.linear.y} z = {msg.linear.z}")
    rospy.loginfo(f"Angular: x = {msg.angular.x}, y = {msg.angular.y} z = {msg.angular.z}")
    
def listener():

    rospy.init_node('teleop_greenbot', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()