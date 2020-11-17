#!/usr/bin/env python
import cv2
import zbar

def main(): 
    scanner = zbar.Scanner()

    cap = cv2.VideoCapture(0)
    width = 960
    height = 544
    cap.set(3, width)
    cap.set(4, height)

    while cap.isOpened():
        ret, image = cap.read()
        view_image = cv2.resize(image, (0,0), fy=0.5, fx=0.5)
        gray = cv2.cvtColor(view_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("image", view_image)

        results = scanner.scan(gray)

        for result in results:
            print(result.data)

        command = chr(cv2.waitKey(1) & 255)

        if command == 'q':
            cap.release()

if __name__ == '__main__':
    main()