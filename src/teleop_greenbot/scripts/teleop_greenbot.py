#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_greenbot')
import rospy
import serial
import time
from geometry_msgs.msg import Twist

class TeleopGreenbot:
    def __init__(self):
        rospy.init_node('teleop_greenbot', anonymous=True)

        rospy.Subscriber("/cmd_vel", Twist, self.handle_twist_message)

        self.udoo_serial = self.initialize_arduino()

    # ------------------------------------------------------------------------------
    @staticmethod
    def initialize_arduino():
        try:
            ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
            ser.flushOutput()

            rospy.loginfo("Connected to Arduino!")

        except Exception as e:

            rospy.logerr(f"Arduino Initialization error: {e}")

        finally:

            return ser

    def handle_twist_message(self, msg):

        rospy.loginfo(f"Linear: x = {msg.linear.x}, y = {msg.linear.y} z = {msg.linear.z}")
        rospy.loginfo(f"Angular: x = {msg.angular.x}, y = {msg.angular.y} z = {msg.angular.z}")

    def sendToArduino(self, msg):

        self.udoo_serial(b"{msg}")

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
# def listener():

#     rospy.init_node('teleop_greenbot', anonymous=True)

#     rospy.Subscriber("/cmd_vel", Twist, callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

if __name__ == '__main__':
    teleop_greenbot = TeleopGreenbot()
    teleop_greenbot.start()