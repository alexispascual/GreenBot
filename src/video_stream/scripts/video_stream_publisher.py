#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys, select, termios, tty

class ImageStreamer:
    def __init__(self):

        # Initialize video_stream node
        rospy.init_node('video_stream', anonymous=True)

        # Initialize Publisher
        self.image_publisher = rospy.Publisher("/image_raw", Image, queue_size=10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize rate of publishing @10Hz
        self.rate = rospy.Rate(10)

        self.key_timeout = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self, key_timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def startVideoCapture(self):

        # Try initializing camera
        try:
            cap = cv2.VideoCapture(0)
            width = 800
            height = 600
            cap.set(3, width)
            cap.set(4, height)

        # Escape errors
        except Exception as e:
            rospy.logerror(f"Error opening the camera: {e}")

        # If no errors encountered, open capture, and convert cv2 image to ROS image
        else:
            while (self.image_publisher.get_num_connections() & cap.isOpened()):
                ret, image = cap.read()
                cv_image = cv2.resize(image, (0,0), fy=0.5, fx=0.5) 

                image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

                # Publish image
                self.image_publisher.publish(image_message)

                # Sleep
                self.rate.sleep()

        finally:
            rospy.loginfo("Subscriber not subscribed anymore! Closing video capture")
            cap.release()
            cv2.destroyAllWindows()

    def start(self):

        while not rospy.is_shutdown():
            # Wait for subscribers
            if self.image_publisher.get_num_connections() == 0:
                rospy.loginfo("Waiting for subscribers...")
                rospy.sleep(1)

            # Start video capture
            else:
                rospy.loginfo("Got a Subscriber! Starting video capture")
                self.startVideoCapture();

if __name__ == '__main__':
    image_streamer = ImageStreamer()
    image_streamer.start()
