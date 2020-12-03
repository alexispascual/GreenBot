#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_greenbot')
import rospy
import serial
import time
from teleop_master.msg import MotionCommand
from std_msgs.msg import Int8

class TeleopGreenbot:
    def __init__(self):
        # Initialize teleop node
        rospy.init_node('teleop_greenbot', anonymous=True)

        # Initialize Master Command subscriber
        self.master_cmd_subscriber = rospy.Subscriber('/master_cmd', Int8, self.handleMasterCommand, queue_size=3)

        # Initialize Arduino Serial comms
        self.udoo_serial = self.initializeArduino()

        # Initialize self.motion_cmd_subscriber property
        self.motion_cmd_subscriber = None

    @staticmethod
    def initializeArduino():
        
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            ser.flushOutput()

            rospy.loginfo("Connected to Arduino!")

        except Exception as e:
            rospy.logerr(f"Arduino Initialization error: {e}")

        else:
            return ser

    def handleMasterCommand(self, msg):

        if msg.data == 0:
            rospy.loginfo("Switching to Stand By mode. Releasing motion command subscriber.")
            if self.motion_cmd_subscriber:
                self.motion_cmd_subscriber.unregister()
                self.motion_cmd_subscriber = None

        elif msg.data == 1: 
            rospy.loginfo("Switching to Manual Teleoperation mode. Spawning motion command subscriber.")
            self.motion_cmd_subscriber = rospy.Subscriber('/cmd_vel', MotionCommand, self.handleMotionCommand, queue_size=10, buff_size=2**16)

        elif msg.data == 2:
            rospy.loginfo("Switching to Autonomous mode. Releasing motion command subscriber.")
            if self.motion_cmd_subscriber:
                self.motion_cmd_subscriber.unregister()
                self.motion_cmd_subscriber = None

    def handleMotionCommand(self, msg):

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