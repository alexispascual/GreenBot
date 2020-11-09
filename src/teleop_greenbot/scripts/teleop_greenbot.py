#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_greenbot')
import rospy
import serial
import time
from geometry_msgs.msg import Twist
from teleop_master.msg import Command

class TeleopGreenbot:
    def __init__(self):
        rospy.init_node('teleop_greenbot', anonymous=True)

        rospy.Subscriber("/cmd_vel", Command, self.handleCommandMessage, queue_size=3, buff_size=2**24)

        self.udoo_serial = self.initializeArduino()

    # ------------------------------------------------------------------------------
    @staticmethod
    def initializeArduino():
        try:
            ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
            ser.flushOutput()

            rospy.loginfo("Connected to Arduino!")

        except Exception as e:

            rospy.logerr(f"Arduino Initialization error: {e}")

        finally:

            return ser

    def handleCommandMessage(self, msg):

        rospy.loginfo(f"Motion: x = {msg.x}, z = {msg.z}")
        rospy.loginfo(f"Mast control: {msg.mast_control}")
        rospy.loginfo(f"Speed: {msg.speed}")

        self.sendToArduino(msg.x, msg.z, msg.mast_control, msg.speed)

    def sendToArduino(self, x, z, mast_control, speed):

        message = f"[{x},{z},{mast_control},{speed}]"

        byte_array = bytearray()
        byte_array.extend(message.encode()) 
        
        self.udoo_serial.write(byte_array)

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
if __name__ == '__main__':
    teleop_greenbot = TeleopGreenbot()
    teleop_greenbot.start()