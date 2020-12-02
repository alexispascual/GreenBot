#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_master')
import rospy
import serial
import time
import yaml
from teleop_master.msg import MotionCommand
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy

class TeleopMaster:
    def __init__(self):
    
        # Initialize teleop_master node    
        rospy.init_node('teleop_master', anonymous=True)

        # Initialize joy message subscribers
        rospy.Subscriber('/joy', Joy, self.handleJoyMessage, queue_size=3, buff_size=2**24)

        # Initialize cmd_vel publisher
        self.motion_cmd_publisher = rospy.Publisher('cmd_vel', MotionCommand, queue_size=10)

        # Initialize master_cmd publisher
        self.master_cmd_publisher = rospy.Publisher('master_cmd', Int8, queue_size=10)

        # Define buttons
        self.teleop_control_button = 9
        self.autonomous_control_button = 8
        self.enable_button = 0
        self.x_axis_index = 5
        self.z_axis_index = 4
        self.mast_button_up = 4
        self.mast_button_down = 5
        self.speed_button_up = 6
        self.speed_button_down = 7
        self.enable_control = False
        self.new_command = False

        # Initialize rover status
        self.control_status = 0
        self.status_change = False

        # Initialize control modes and default speed
        self.x = 0
        self.z = 0
        self.speed = 20
        self.turning_speed = 102
        self.mast_control = 0

        # Initialize rate @ 10Hz
        self.rate = rospy.Rate(10)

    def getMotionCommand(self):

        command = MotionCommand()

        command.x = int(self.x)
        command.z = int(self.z)
        command.mast_control = int(self.mast_control)
        command.speed = float(self.speed)

        return command

    def handleJoyMessage(self, joy_msg):

        # Toggle Start/Stop for Greenbot Teleop mode
        if joy_msg.buttons[self.teleop_control_button]:
            self.status_change = True

            if self.control_status == 1:
                self.control_status = 0

            else:
                self.control_status = 1

        elif joy_msg.buttons[self.autonomous_control_button]:
            self.status_change = True

            if self.control_status == 2:
                self.control_status = 0 

            else:
                self.control_status = 2

        # Make sure the enable button is pressed before executing commands.
        elif joy_msg.buttons[self.enable_button]:
            self.new_command = True
            # Get directions for x and z axes
            self.x = joy_msg.axes[self.x_axis_index]
            self.z = joy_msg.axes[self.z_axis_index]

            # Get directions for mast control
            if joy_msg.buttons[self.mast_button_up]:
                self.mast_control = 1
            elif joy_msg.buttons[self.mast_button_down]:
                self.mast_control = -1
            else:
                self.mast_control = 0

            # Increase/decrese Greenbot speed while limiting ranges
            if joy_msg.buttons[self.speed_button_up]:

                self.speed = 22
                rospy.loginfo(f"Greenbot at {(self.speed/255)*100:.2f}% speed!")

                if self.speed >= 255:
                    self.speed = 255

            elif joy_msg.buttons[self.speed_button_down]:

                self.speed = 102
                rospy.loginfo(f"Greenbot at {(self.speed/255)*100:.2f} speed!")

                if self.speed < 0:
                    self.speed = 0

        # If enable button is not pressed, don't do anything                
        else:
            self.new_command = True

            self.x = 0
            self.z = 0
            self.mast_control = 0

    def teleopMode(self):
        rospy.loginfo("Starting teleop mode!")

        while self.master_cmd_publisher.get_num_connections() and (self.control_status == 1):
            if self.new_command:
                # Get command string
                command_message = self.getMotionCommand()

                print(command_message)

                # Publish command message
                self.motion_cmd_publisher.publish(command_message)

                self.new_command = False
                
            self.rate.sleep()
        else:
            rospy.loginfo("Exiting teleop mode...")

    def autonomousMode(self):

        rospy.loginfo("Starting autonomous mode!")

        while self.master_cmd_publisher.get_num_connections() and (self.control_status == 2):
            # TODO: Figure out what master does when rover is autonomous
            rospy.loginfo("Currently in autonomous mode...")
            rospy.sleep(1)
        else:
            rospy.loginfo("Exiting autonomous mode...")

    def standByMode(self):

        while self.master_cmd_publisher.get_num_connections() and (self.control_status == 0):
            rospy.loginfo("Greenbot is in Standby mode. Press 'Start' to begin teleop mode.")
            rospy.loginfo("To begin Autonomous mode, line up the rover to the first QR code and press 'Back'")
            rospy.sleep(1)

        else:
            rospy.loginfo("Exiting Standby mode...")

    def greenbotSubscribed(self):

        while self.master_cmd_publisher.get_num_connections():
            if self.status_change:
                # Greenbot in Teleop mode
                self.status_change = False

                if self.control_status == 0:
                    self.master_cmd_publisher.publish(0)
                    self.standByMode()

                # Greenbot in Teleop mode
                elif self.control_status == 1:
                    self.master_cmd_publisher.publish(1)
                    self.teleopMode()

                # Greenbot in Autonomous mode        
                elif self.control_status == 2:
                    self.master_cmd_publisher.publish(2)
                    self.autonomousMode()

            self.rate.sleep()

    def start(self):

        # Initialize Publisher
        while not rospy.is_shutdown():

            # Wait for subscribers
            if self.master_cmd_publisher.get_num_connections() == 0:
                rospy.loginfo("Waiting for Geenbot to subscribe...")
                rospy.sleep(1)
            
            else:
                # Once Greenbot is subscribed, start publishing
                self.control_status = 0
                rospy.loginfo("Greenbot subscribed! Starting Greenbot in Standby mode")
                self.greenbotSubscribed()

if __name__ == '__main__':
    teleop_master = TeleopMaster()
    teleop_master.start()
