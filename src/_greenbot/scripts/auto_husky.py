#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Polygon
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
import numpy as np
import serial
import subprocess
import re
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import geometry_msgs
import math
import time as tm
# import IntList

import roslib
roslib.load_manifest("greenbot")
np.seterr(divide='ignore', invalid='ignore')


# ------------------------------------------------------------------------------
# Autonomous Navigation CLASS
# ------------------------------------------------------------------------------
class AutoHusky:
    def __init__(self):
        rospy.init_node('auto_husky')  # Initiate teleop node

        # Subscriber Initiations
        #rospy.Subscriber('person_detection/person', Polygon, self.handle_polygon)
        # rospy.Subscriber('my_cam')
        rospy.Subscriber('/isNotRepeated_QR', Bool, self.handle_qr)
        #rospy.Subscriber('husky_velocity_controller/odom', Odometry, self.handle_odom)
        rospy.Subscriber('joy', Joy, self.handle_joy)
        #rospy.Subscriber('Range_Data', String, self.handle_range)
        rospy.Subscriber('repeated_QR', Bool, self.handle_qr)


        # Publisher Initiations
        self.vel_cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.detect_qr = False
        self.polygon_data = None
        self.joy_data = None
        self.range_data = None
        self.deadman = 0  # Auto testing deadman button value
        self.lab = 0
        self.z = 0

        self.state = None
        self.possible_states = ['Row', 'Enter', 'Exit', 'Aisle']
        self.qr_count = 0

        # Open the serial port
        #self.arduino = serial.Serial(self.get_usb_info(), 9600)

        # Determine initial state
        #self.determine_state()

        # Center the Husky in the row
        #self.center_husky()

        return

# ------------------------------------------------------------------------------
    @staticmethod
    def get_usb_info():
        # regular expression for lsusb output
        device_re = re.compile(r"Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+):\s+"
                               r"ID\s+(?P<id>\w+:\w+)\s*(?P<tag>.+)$", re.I | re.MULTILINE)
        usb_info = subprocess.check_output('lsusb')
        device = 'dev/bus/usb/'

        for info in usb_info.split('\n'):
            if info:
                dev_info = device_re.match(info)

                dev_info['devices'] = '/dev/bus/usb/{bus}/{dev}'.format(bus=dev_info.pop('bus'),
                                                                        dev=dev_info.pop('device'))
        return device

# ------------------------------------------------------------------------------
    def handle_joy(self, joy_data):
        self.lab = joy_data.buttons[4]
        self.z = joy_data.buttons[5]
        self.joy_data = joy_data
        return

# ------------------------------------------------------------------------------
    def handle_polygon(self, poly_data):
        self.polygon_data = poly_data
        return

# ------------------------------------------------------------------------------
    def handle_range(self, range_data):
        self.range_data = np.array([range_data['FR'], range_data['FL'],
                                    range_data['BR'], range_data['BL']])
        return

# ------------------------------------------------------------------------------
    def handle_odom(self, odom_data):
        odom = Odometry()
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "husky"
        t.child_frame_id = odom_data.child_frame_id
        t.transform.translation.x = odom_data.pose.pose.position.x
        t.transform.translation.y = odom_data.pose.pose.position.y
        t.transform.translation.z = 0.0
        #q = tf_conversions.transformations.quaternion_from_euler(0, 0, odom.pose.pose.orientation.theta)
        q = odom_data.pose.pose.orientation
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        br.sendTransform(t)
        return

# ------------------------------------------------------------------------------
    def handle_qr(self, data):
        self.detect_qr = data
        return

# ------------------------------------------------------------------------------
    # Check if the Husky has moved enough to be considered in a different state
    def determine_state(self):
        if self.range_data.all() < 25:
            self.state = 'Row'
        elif not self.range_data[0:2] < 25:
            self.state = 'Exit'
        elif not self.range_data[2:4] < 25:
            self.state = 'Enter'
        elif self.range_data.all() > 25:
            self.state = 'Aisle'
        return

# ------------------------------------------------------------------------------
    def drive(self):
        self.qr_count = 0
        # In a row and driving to detect qr codes
        if self.state == 'Row':
            for count in range(1):  # Count to 2, 1 for entering, 1 for exiting
                while self.qr_count < 5:
                    while self.detect_qr == False:
                        self.drive_straight()  # publish drive command

                    if self.detect_qr == True:  # Stop at qr code
                        self.qr_count += 1  # increment qr code count
                        rospy.sleep(15)  # 15 second pause
                        self.detect_qr = False

                if count == 0:
                    self.turn_deg(math.radians(180))  # turn 180 degrees at end of row (5 qr codes)
                self.qr_count = 0  # Reset count for trip back to end
            self.state = 'Exit'

        while self.state == 'Exit':  # While Husky is entering the aisle
            while self.range_data[2:4] < 25:  # Wait for the back sensors to lose the platform
                self.drive_straight()
            # TODO possibly add extra drive time to clear platform?
            self.turn_deg(math.radians(90))
            self.state = 'Aisle'

        while self.state == 'Aisle':  # In the main aisle
            while self.range_data[1] < 25:  # Front Left sensor detects platform
                # Start passing the platform
                self.drive_straight()
            while self.range_data[3] < 25:  # Back left sensor detects platform
                # Wait for back of Husky to pass platform
                self.drive_straight()
            self.turn_deg(math.radians(90))  # Turn into row
            self.state = 'Enter'

        while self.state == 'Enter':
            while self.range_data[2:4] > 25:
                self.drive_straight()
            self.state = 'Row'
        return

# ------------------------------------------------------------------------------
    # Ensure the Husky's front and back sensors see the same distance
    def center_husky(self):
        while self.range_data[0]-self.range_data[2] != 0:
            if self.range_data[0] > self.range_data[2]:
                self.turn_deg(math.radians(1))
            else:
                self.turn_deg(math.radians(-1))
        return

# ------------------------------------------------------------------------------
    def drive_straight(self, time=None):
        start_time = tm.time()
        #stop_time = 15
        command = Twist()
        command.linear.x = 0.1
        command.angular.z = 0
        if time is not None:
            while tm.time() - start_time < time:
                while self.detect_qr is True:
                    pass
                self.vel_cmd_pub.publish(command)
        else:
            self.vel_cmd_pub.publish(command)
        return

# ------------------------------------------------------------------------------
    # Rotate the Husky arg degrees in place
    def turn_deg(self, degrees, z=False):
        command = Twist()
        start_time = tm.time()
        rad = math.radians(degrees)
        if degrees > 0:
            turntime = 16
        if degrees < 0:
            turntime = 15

        if z is True:
            if rad > 0:
                command.linear.x = 0.0
                command.angular.z = -0.1  # Turn command, -ve is Left, +ve is Right
            elif rad < 0:
                command.linear.x = 0.0
                command.angular.z = 0.1
            while tm.time() - start_time < turntime:
                self.vel_cmd_pub.publish(command)  # Publish command
            return

        try:
            command.linear.x = 0.0
            command.angular.z = -0.1  # Turn command, -ve is Left, +ve is Right
            turn_complete = False

            while turn_complete is False:
                self.vel_cmd_pub.publish(command)  # Publish command
                # Get transform b/w current frame and start frame
                #test = self.tf_buffer.lookup_transform('base_link','base_link', start_time)
                #print(test)
                trans = self.tf_buffer.lookup_transform_full('front_right_wheel_link',
                                                             rospy.Time.now(),
                                                            'front_right_wheel_link',
                                                             start_time,
                                                             'base_link',
                                                             rospy.Duration(1))
                print (trans)
                if trans == [0, 0, 0, rad]:  # Turn complete
                    turn_complete = True

        except Exception as error:
            print (error)
        return

# ------------------------------------------------------------------------------
    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #while self.joy_data.buttons[self.deadman] == 1:  # Deadman button for auto testing
                # self.determine_state()  # Use data from ultra sensors, front camera
                #self.drive()
            # Testing functionality
            while self.joy_data.buttons[6] == 1:
            #while self.lab == 1:
                self.drive_straight(20)
                self.turn_deg(180)
                self.drive_straight(20)
                self.turn_deg(90)
                rospy.sleep(30)
                self.turn_deg(-90)
                rospy.sleep(30)
                self.turn_deg(180)
                rospy.sleep(30)
                self.drive_straight(20)

            while self.z == 1:
                self.turn_deg(-90, z=True)
                self.drive_straight(18)
                self.turn_deg(-90, z=True)                
                self.drive_straight(50) # Increase
                self.turn_deg(90, z=True) # Increase by 1 second?
                self.turn_deg(90, z=True)
                self.drive_straight(50)
                self.turn_deg(-90, z=True)
                self.drive_straight(18)
                self.turn_deg(-90, z=True)
                self.drive_straight(50)
                self.turn_deg(90, z=True)
                self.turn_deg(90, z=True)
                self.drive_straight(50)

        return

# ------------------------------------------------------------------------------
    def pub_mast_cmd(self, command):
        # encode in the appropriate format and send
        self.arduino.write(command.encode())
        return

    def store_range_data(self):
        return

# ------------------------------------------------------------------------------
# END OF Autonomous Navigation CLASS
# ------------------------------------------------------------------------------


if __name__ == '__main__':
    cmd = AutoHusky()
    cmd.start()

