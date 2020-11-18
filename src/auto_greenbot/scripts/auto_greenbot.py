#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

class AutonomousGreenbot:
    
    def __init__(self, arg):

        # Initialize QR code subscriber
        rospy.Subscriber("/qr_code_decoded", String, self.determineState, queue_size=10, buff_size=2**24)

        self.state = 0

    def determineState(self, msg):
        """ 
        Receive QR code meaning and determine state. 
        Execute commands depending on curent state 
        """

        
        if self.state == -1: # Halt. Stop everything
            self.haltGreenbot()

        elif self.state == 0: # Standby, await command
            self.standBy()
        
        elif self.state == 1: # Found green QR code, take image, then drive forward
            self.takeImage()
            self.driveForward()

        elif self.state == 2 # Found black QR code, turn corner
            self.turnCorner()

        elif self.state == 3 # Found red QR code, turn in to row
            self.turnInToRow()

        elif self.state == 4 # Found pink QR code, turn around
            self.turnAround()

    def haltGreenbot(self):
        """
        Stop everything
        """

    def standBy(self):
        """
        Stand by 
        """

    def takeImage(self):
        """
        Take image. Probably just stop. I don't know if we could command the camera
        """

    def driveForward(self):
        """
        Drive forward 
        """

    def turnCorner(self):
        """
        Turn corner
        """

    def turnInToRow(self):
        """
        Turn in to row
        """
        
    def turnAround(self):
        """
        Turn around
        """

    def start():
        pass

if __name__ == '__main__':
    auto_greenbot = AutonomousGreenbot()
    auto_greenbot.start
