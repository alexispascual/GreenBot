#!/usr/bin/env python
import cv2
import zbar
import rospy
from std_msgs.msg import String

class QRCodeScanner():
    def __init__(self):

        rospy.init_node('qr_code', anonymous=False)

        self.qr_code_publisher = rospy.Publisher("/qr_code_decoded", String, queue_size = 10)

        self.rate = rospy.Rate(100)

        self.scanner = zbar.Scanner()

    def startScanning(self):

        try:
            cap = cv2.VideoCapture(0) # TODO: Figure out if there's 2 cameras
            width = 800
            height = 600
            cap.set(3, width)
            cap.set(4, height)

        except cv2.error as e:

            rospy.logerror(f"Error opening the camera: {e}")

        else:
            while(self.qr_code_publisher.get_num_connections() & cap.isOpened()):
                ret, image = cap.read()
                view_image = cv2.resize(image, (0,0), fy=0.5, fx=0.5)
                gray = cv2.cvtColor(view_image, cv2.COLOR_BGR2GRAY)

                results = self.scanner.scan(gray)

                for result in results:
                    if result.data:
                        self.qr_code_publisher.publish(result.data.decode("utf-8"))

                self.rate.sleep()

        finally:
            rospy.loginfo("Subscriber not subscribed anymore. Closing video capture")
            cap.release()
            cv2.destroyAllWindows()

    def start(self):

        while not rospy.is_shutdown():
            # Wait for subscribers
            if self.qr_code_publisher.get_num_connections() == 0:
                rospy.loginfo("Waiting for subscribers...")
                rospy.sleep(1)

            # Start video capture
            else:
                rospy.loginfo("Got a Subscriber! Starting qr code scanning")
                self.startScanning();

if __name__ == '__main__':
    qr_scanner = QRCodeScanner()
    qr_scanner.start()