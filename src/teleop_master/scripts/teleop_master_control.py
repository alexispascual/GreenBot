#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_master')
import rospy
import serial
import time
import constants as c
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
        self.teleop_control_button = c.START_BUTTON
        self.autonomous_control_button = c.BACK_BUTTON
        self.enable_button = c.X_BUTTON
        self.x_axis_index = c.UP_DOWN_AXIS
        self.z_axis_index = c.LEFT_RIGHT_AXIS
        self.mast_button_up = c.LB_BUTTON
        self.mast_button_down = c.RB_BUTTON
        self.speed_button_up = c.LT_BUTTON
        self.speed_button_down = c.RT_BUTTON
        self.enable_control = False
        self.new_command = False

        # Initialize rover status
        self.control_status = c.STANDBY

        # Initialize control modes and default speed
        self.x = 0
        self.z = 0
        self.speed = 127
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
            if self.control_status == c.TELEOP_MANUAL:
                rospy.loginfo("GreenBot stading by...")
                self.control_status = c.STANDBY

            elif self.control_status == c.STANDBY:
                rospy.loginfo("Starting teleop mode!")
                self.control_status = c.TELEOP_MANUAL

            return

        elif joy_msg.buttons[self.autonomous_control_button]:
            if self.control_status != c.AUTONOMOUS:
                rospy.loginfo("Starting autonomous mode!")
                self.control_status = c.AUTONOMOUS

            return

        # Make sure the enable button is pressed before executing commands.
        try:
            if joy_msg.buttons[self.enable_button]:

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

                    self.speed += 1
                    rospy.loginfo(f"Greenbot at {(self.speed/255)*100:.2f}% speed!")

                    if self.speed >= 255:
                        self.speed = 255

                elif joy_msg.buttons[self.speed_button_down]:

                    self.speed -= 1
                    rospy.loginfo(f"Greenbot at {(self.speed/255)*100:.2f} speed!")

                    if self.speed < 0:
                        self.speed = 0

            # If enable button is not pressed, don't do anything                
            else:
                self.x = 0
                self.z = 0
                self.mast_control = 0

        except Exception as e:
            rospy.logerr(f"Encountered joy message handling error {e}")

        finally:
            self.new_command = True
        
        return

    def TeleopMode(self):
        while (self.control_status == 1):
            if self.new_command:
                # Get command string
                command_message = self.getMotionCommand()

                print(command_message)

                # Publish command message
                self.motion_cmd_publisher.publish(command_message)

                self.new_command = False
                
            self.rate.sleep()

    def AutonomousMode(self):
        while (self.control_status == 2):
            # TODO: Figure out what master does when rover is autonomous
            rospy.sleep(5)
            pass


    def start(self):

        # Initialize Publisher
        while not rospy.is_shutdown():

            # Wait for subscribers
            if self.master_cmd_publisher.get_num_connections() == 0:
                rospy.loginfo("Waiting for Geenbot to subscribe...")
                rospy.sleep(1)

            # Once Greenbot is subscribed, start publishing
            elif (self.master_cmd_publisher.get_num_connections() & self.control_status == c.STANDBY):
                self.master_cmd_publisher.publish(c.STANDBY)
                rospy.loginfo("Greenbot is in Standby mode. Press 'Start' to begin teleop mode.")
                rospy.loginfo("To begin Autonomous mode, line up the rover to the first QR code and press 'Back'")
                rospy.sleep(1)

            # Greenbot in Teleop mode
            elif (self.master_cmd_publisher.get_num_connections() & self.control_status == c.TELEOP_MANUAL):
                self.master_cmd_publisher.publish(c.TELEOP_MANUAL)
                self.TeleopMode()

            # Greenbot in Autonomous mode        
            elif(self.master_cmd_publisher.get_num_connections() & self.control_status == c.AUTONOMOUS):
                self.master_cmd_publisher.publish(c.AUTONOMOUS)
                self.AutonomousMode()



if __name__ == '__main__':
    teleop_master = TeleopMaster()
    teleop_master.start()