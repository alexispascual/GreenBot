#!/usr/bin/env python
import cv2
import zbar
import rospy
import json
from std_msgs.msg import String

class QRCodeScanner():
    def __init__(self):

        rospy.init_node('qr_code', anonymous=False)

        self.qr_code_publisher = rospy.Publisher('/qr_code_decoded', String, queue_size = 10)

        self.rate = rospy.Rate(100)

        self.scanner = zbar.Scanner()

    def startScanning(self):

        last_read_code = 0

        try:
            cap = cv2.VideoCapture(0) # TODO: Figure out if there's 2 cameras

            # Can be configured to any width/height
            # For now, just using default dimensions
            width = cap.get(3) 
            height = cap.get(4)

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
                    if result.data and last_read_code != result.data:
                        last_read_code = result.data

                        json_parsed = json.loads(result.data.decode("utf-8"))

                        if json_parsed.get('S', None):
                            rospy.loginfo(result.data.decode("utf-8"))
                            self.qr_code_publisher.publish(result.data.decode("utf-8"))

                self.rate.sleep()

        finally:
            rospy.loginfo("Subscriber not subscribed anymore. Closing video capture")
            cap.release()
            cv2.destroyAllWindows()

    def publishString(self):
        while  True:
            
            i = 0

            while i < 7:
                sample_string = '{"S":' + str(i) + ', "I": 1}'
                print(sample_string)
                self.qr_code_publisher.publish(sample_string)
                rospy.sleep(2)
                i += 1

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