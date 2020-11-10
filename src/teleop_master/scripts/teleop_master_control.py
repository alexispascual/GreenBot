#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_master')
import rospy
import serial
import time
from teleop_master.msg import Command
from sensor_msgs.msg import Joy

class TeleopMaster:
    def __init__(self):
    
        # Initialize teleop_master node    
        rospy.init_node('teleop_master', anonymous=True)

        # Initialize joy message subscribers
        rospy.Subscriber("/joy", Joy, self.handleJoyMessage, queue_size=3, buff_size=2**24)

        # Initialize cmd_vel publisher
        self.command_publisher = rospy.Publisher('cmd_vel', Command, queue_size=10)

        # Define buttons
        self.start_control = False
        self.start_control_button = 9
        self.enable_button = 0
        self.x_axis_index = 5
        self.z_axis_index = 4
        self.mast_button_up = 4
        self.mast_button_down = 5
        self.speed_button_up = 6
        self.speed_button_down = 7
        self.enable_control = False
        self.new_command = False

        # Initialize control modes and default speed
        self.x = 0
        self.z = 0
        self.speed = 127
        self.mast_control = 0

    def getCommand(self):

        command = Command()

        command.x = int(self.x)
        command.z = int(self.z)
        command.mast_control = int(self.mast_control)
        command.speed = float(self.speed)

        return command

    def handleJoyMessage(self, joy_msg):

        # Toggle Start/Stop for Greenbot Teleop mode
        if joy_msg.buttons[self.start_control_button]:
            if not self.start_control:
                rospy.loginfo("Starting teleop mode!")
                self.start_control = True
            else:
                rospy.loginfo("Stopping teleop mode!")
                self.start_control = False

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

    def start(self):

        # Initialize rate @ 10Hz
        rate = rospy.Rate(10)

        # Initialize Publisher
        while not rospy.is_shutdown():

            # Wait for subscribers
            if self.command_publisher.get_num_connections() == 0:
                rospy.loginfo("Waiting for subscribers...")
                rospy.sleep(1)

            # Once Greenbot is subscribed, start publishing
            else:
                if not self.start_control:
                    rospy.loginfo("Control is currently disabled. Press Start to begin teleop mode.")
                    rospy.sleep(1)

                # Check if there's a new command
                else:
                    if self.new_command:
                        # Get command string
                        command_message = self.getCommand()

                        print(command_message)

                        # Publish command message
                        self.command_publisher.publish(command_message)

                        self.new_command = False

                rate.sleep()

if __name__ == '__main__':
    teleop_master = TeleopMaster()
    teleop_master.start()