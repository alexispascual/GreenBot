#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

class AutonomousGreenbot:
    
    def __init__(self):

        # Initialize QR code subscriber
        rospy.Subscriber("/qr_code_decoded", String, self.determineState, queue_size=10, buff_size=2**24)

        self.udoo_serial = self.initializeArduino()

        self.state = 0

        self.drive_forward_duration = 90
        self.imaging_duration = 60
        self.turn_maneuver_forward_duration = 15
        self.turn_maneuver_turn_duration = 90
        self.turn_around_duration = 180
        self.gb_default_speed = 127
        self.gb_slow_speed = 32

    def initializeArduino(self):

        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            ser.flushOutput()

            rospy.loginfo("Connected to Arduino!")

        except Exception as e:

            rospy.logerr(f"Arduino Initialization error: {e}")

        finally:

            return ser

    def determineState(self, msg):
        """ 
        Receive QR code meaning and determine state. 
        Execute commands depending on curent state 

        TODO: parse message to determine state. Probably a good idea
        to use some function instead of a massive if else
        """

        switch = {
            -1: haltGreenbot,
            0: standBy,
            1: driveForward,
            2: takeImage,
            3: turnCorner,
            4: turnInToRow,
            5: turnAround,
            6: endOperations,
        }

        method = getattr(switch.get(self.state))
        method()

        # if self.state == -1: # Halt. Stop everything
        #     self.haltGreenbot()

        # elif self.state == 0: # Standby, await command
        #     self.standBy()
        
        # elif self.state == 1: # Found white QR code, drive forward
        #     # Perform alignment to plant box?
        #     self.driveForward()

        # elif self.state == 2: # Found green QR code, take image, then drive forward
        #     self.takeImage()
        #     self.driveForward()

        # elif self.state == 3: # Found black QR code, turn corner
        #     self.turnCorner()

        # elif self.state == 4: # Found red QR code, turn in to row
        #     self.turnInToRow()

        # elif self.state == 5: # Found pink QR code, turn around
        #     self.turnAround()

        # elif self.state == 6: # Found blue QR code, end operations
        #     self.endOperations()

    def haltGreenbot(self):
        """
        Stop everything
        """
        self.sendToArduino(0, 0, 0, 0)

    def standBy(self):
        """
        Stand by 
        """
        self.sendToArduino(0, 0, 0, gb_default_speed)

    def takeImage(self):
        """
        Take image. Probably just stop. I don't know if we could command the camera
        """
        self.sendToArduino(0, 0, 0, gb_default_speed)
        rospy.sleep(self.imaging_duration)

    def driveForward(self):
        """
        Drive forward 
        """
        self.sendToArduino(1, 0, 0, gb_default_speed)
        rospy.sleep(self.drive_forward_duration)
        self.sendToArduino(1, 0, 0, gb_slow_speed)

    def turnCorner(self):
        """
        Turn corner
        """
        self.sendToArduino(1, 0, 0, gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        self.sendToArduino(0, -1, 0, gb_slow_speed)
        rospy.sleep(self.turn_maneuver_turn_duration)

        self.sendToArduino(1, 0, 0, gb_slow_speed)

    def turnInToRow(self):
        """
        Turn into row
        Maneuver distances?
        """
        self.sendToArduino(1, 0, 0, gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        self.sendToArduino(0, -1, 0, gb_slow_speed)
        rospy.sleep(self.turn_maneuver_turn_duration)

        self.sendToArduino(1, 0, 0, gb_slow_speed)

    def turnAround(self):
        """
        Turn around
        """
        self.sendToArduino(1, 0, 0, gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        self.sendToArduino(0, -1, 0, gb_slow_speed)
        rospy.sleep(self.turn_around_duration)

        self.sendToArduino(1, 0, 0, gb_slow_speed)

    def endOperations(self):
        """
        TODO: End operations
        """
        self.sendToArduino(0, 0, 0, 0)

    def sendToArduino(self, x, z, mast_control, speed):
        """
        Send message to Arduino
        """
        message = f"[{x},{z},{mast_control},{speed}]"

        byte_array = bytearray()
        byte_array.extend(message.encode()) 
        
        self.udoo_serial.write(byte_array)

    def start():
        rospy.spin()

if __name__ == '__main__':
    auto_greenbot = AutonomousGreenbot()
    auto_greenbot.start
