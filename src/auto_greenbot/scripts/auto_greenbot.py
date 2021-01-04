#!/usr/bin/env python
import rospy
import serial
import json
from std_msgs.msg import String
from std_msgs.msg import Int8

class AutonomousGreenbot:
    
    def __init__(self):

        # Initialize node for autonomy
        rospy.init_node('autonomous_greenbot', anonymous=True)

        # Initialize Master Command subscriber
        self.master_cmd_subscriber = rospy.Subscriber('/master_cmd', Int8, self.handleMasterCommand, queue_size=3)

        # Initialize Arduino Serial comms
        self.udoo_serial = self.initializeArduino()

        # Initialize state as Stand by
        self.state = 0
        self.qr_index = 0

        # Initialize default durations
        self.drive_forward_duration = 6.0
        self.imaging_duration = 7.0
        self.turn_maneuver_forward_duration = 6
        self.turn_maneuver_turn_duration = 2.8
        self.turn_around_duration = 5.6

        # Define forward and slow speeds    
        self.gb_default_speed = 12
        self.gb_slow_speed = 10
        self.gb_turning_speed = 40

        # Initialize qr_subscriber object
        self.qr_subscriber = None

        # Initialize state definitions
        self.switch = {
                -1:'haltGreenbot',
                0: 'standBy',
                1: 'driveForward',
                2: 'takeImage',
                3: 'turnCorner',
                4: 'turnInToRow',
                5: 'turnAround',
                6: 'endOperations',
            }

    @staticmethod
    def initializeArduino():

        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            ser.flushOutput()

            rospy.loginfo("Connected to Arduino!")

        except serial.SerialException as e:
            rospy.logerr(f"Arduino Initialization error: {e}")

        else:
            return ser

    def handleMasterCommand(self, msg):

        if msg.data == 0:

            rospy.loginfo("Switching to Stand By mode. Releasing QR code subscriber.")
            self.standBy()

            if self.qr_subscriber:
                self.qr_subscriber.unregister()
                self.qr_subscriber = None

        elif msg.data == 1: 

            rospy.loginfo("Switching to Manual Teleoperation mode. Releasing QR subscriber.")
            self.standBy()

            if self.qr_subscriber:
                self.qr_subscriber.unregister()
                self.qr_subscriber = None

        elif msg.data == 2:
            
            rospy.loginfo("Switching to Autonomous mode. Spawning QR subscriber.")
            self.standBy()
            
            # Initialize QR code subscriber
            self.qr_subscriber = rospy.Subscriber('/qr_code_decoded', String, self.determineState, queue_size=10, buff_size=2**16)

    def determineState(self, msg):
        """ 
        Receive QR code meaning and determine state. 
        Execute commands depending on curent state 
        """
        try: 
            json_parsed = json.loads(msg.data)
            self.state = json_parsed.get('S', None)
            self.qr_index = json_parsed.get('I', None)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing JSON object: {e}")

        else:

            if self.state is not None and self.qr_index is not None:

                method = getattr(self, self.switch.get(self.state))
                method()

            else:
                rospy.loginfo("Invalid QR code read!")

    def haltGreenbot(self):
        """
        Stop everything
        """

        rospy.loginfo("Halting!")
        self.sendToArduino(0, 0, 0, 0)

    def standBy(self):
        """
        Stand by 
        """

        rospy.loginfo("Standing by...")
        self.sendToArduino(0, 0, 0, self.gb_default_speed)

    def takeImage(self):
        """
        Take image. Probably just stop. I don't know if we could command the camera
        """

        rospy.loginfo("Taking an image...")
        self.sendToArduino(0, 0, 0, self.gb_default_speed)
        rospy.sleep(self.imaging_duration)

        rospy.loginfo("Continuing on...")
        self.driveForward()

    def driveForward(self):
        """
        Drive forward 
        """

        rospy.loginfo("Driving forward with auto steering...")
        self.sendToArduino(2, 0, 0, self.gb_default_speed)

    def turnCorner(self):
        """
        Turn corner
        """

        rospy.loginfo("Found end of row!")
        rospy.loginfo("Clearing platform...")
        self.sendToArduino(1, 0, 0, self.gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        rospy.loginfo("Turning into aisle...")
        self.sendToArduino(0, -1, 0, self.gb_turning_speed)
        rospy.sleep(self.turn_maneuver_turn_duration)

        rospy.loginfo("Creeping forward to find red QR code...")
        self.sendToArduino(1, 0, 0, self.gb_slow_speed)

    def turnInToRow(self):
        """
        Turn into row
        Maneuver distances?
        """

        rospy.loginfo("Found red QR code! Turning into row...")
        rospy.loginfo("Clearing platform...")
        self.sendToArduino(1, 0, 0, self.gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        rospy.loginfo("Turning into row...")
        self.sendToArduino(0, -1, 0, self.gb_turning_speed)
        rospy.sleep(self.turn_maneuver_turn_duration)

        rospy.loginfo("Entering row to execute distance correction")
        self.sendToArduino(1, 0, 0, self.gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        rospy.loginfo("Executing distance correction...")
        self.sendToArduino(4, 0, 0, self.gb_slow_speed)

        rospy.loginfo("Creeping forward to find first QR code...")
        self.sendToArduino(2, 0, 0, self.gb_slow_speed)
        

    def turnAround(self):
        """
        Turn around
        """

        rospy.loginfo("Found pink QR code! Turning around...")
        self.sendToArduino(1, 0, 0, self.gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        self.sendToArduino(0, -1, 0, self.gb_turning_speed)
        rospy.sleep(self.turn_around_duration)

        rospy.loginfo("Entering row to execute distance correction")
        self.sendToArduino(1, 0, 0, self.gb_slow_speed)
        rospy.sleep(self.turn_maneuver_forward_duration)

        rospy.loginfo("Executing distance correction...")
        self.sendToArduino(4, 0, 0, self.gb_slow_speed)

        rospy.loginfo("Creeping forward to find 1st QR code")
        self.sendToArduino(2, 0, 0, self.gb_slow_speed)

    def endOperations(self):
        """
        End operations
        """

        rospy.loginfo("Ending operations. Switch to manual mode!")
        self.sendToArduino(0, 0, 0, self.gb_default_speed)

    def sendToArduino(self, x, z, mast_control, speed):
        """
        Send message to Arduino
        """

        message = f"[{x},{z},{mast_control},{speed}]"

        byte_array = bytearray()
        byte_array.extend(message.encode()) 
        
        self.udoo_serial.write(byte_array)

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    auto_greenbot = AutonomousGreenbot()
    auto_greenbot.start()
