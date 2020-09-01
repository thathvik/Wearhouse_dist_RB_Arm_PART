# import the necessary packages
# to connect to the Bluetooth enter tthe following codes to find it and connect
# $         hcitool scan
# connect to the required Bluetooth using the right MAC ID
# $         sudo rfcomm connect hci0 xx:xx:xx:xx:xx:xx

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import timeit
from scipy import signal
import matplotlib.pyplot as plt

import RPi.GPIO as GPIO

import serial

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)  # Gripper
GPIO.setup(13, GPIO.OUT)  # Rot_x
GPIO.setup(16, GPIO.OUT)  # Rot_z

rotz = 16
rotx = GPIO.PWM(13, 50)
gr = GPIO.PWM(12, 50)

blue = serial.Serial("/dev/rfcomm0", baudrate=9600)
print("Bluetooth connected")


def duty(angle):
    return angle * 5 / 90 + 2.5


def search(angle=90, add=1):
    servo_pwm(rotz, duty(angle), 50)
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())
    xn = np.zeros([500])
    xm = np.zeros([1])
    greenLower = (20, 20, 53)
    greenUpper = (64, 255, 255)
    pts = deque(maxlen=args["buffer"])
    # if a video path was not supplied, grab the reference
    # to the webcam
    if not args.get("video", False):
        vs = VideoStream(src=0).start()
    # otherwise, grab a reference to the video file
    else:
        vs = cv2.VideoCapture(args["video"])
    # allow the camera or video file to warm up
    time.sleep(2.0)

    while True:
        if angle == 125:
            add = -5
        elif angle == 35:
            add = 5
        angle += add
        servo_pwm(rotz, duty(angle), 10)
        time.sleep(0.01)
        # grab the current frame
        frame = vs.read()
        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                xn = np.delete(xn, 0)
                xn = np.append(xn, x)
                fs = 300
                fc = 1
                x_old = x
                w = fc / (fs / 2)
                b, a = signal.butter(5, w, 'low')
                output = signal.filtfilt(b, a, xn)
                x = np.average(xn[480:500])
                print(x, x_old)
                xm = np.append(xm, x)
                if abs(x - 300) < 20:
                    break
        # update the points queue
        pts.appendleft(center)
        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            print(xn)
            print(xn.shape)
            plt.plot(xm, label='x')
            plt.show()
            break

    if not args.get("video", False):
        vs.stop()
    # otherwise, release the camera
    else:
        vs.release()
    # close all windows
    cv2.destroyAllWindows()
    return x, add


def servo_pwm(pin, duty, pulse):
    on = 20 * duty / 100000
    off = -on + 20 / 1000
    for i in range(pulse):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(on)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(off)


def grip(angle=90):
    servo_pwm(rotz, duty(angle), 100)
    rotx.start(duty(90))
    gr.start(duty(100))
    time.sleep(1)
    rotx.ChangeDutyCycle(duty(0))
    time.sleep(1)
    gr.ChangeDutyCycle(duty(180))
    time.sleep(0.5)
    rotx.ChangeDutyCycle(duty(90))
    time.sleep(0.5)


def drop():
    rotx.ChangeDutyCycle(duty(180))
    time.sleep(1)
    gr.ChangeDutyCycle(duty(100))
    time.sleep(1)
    rotx.ChangeDutyCycle(duty(90))
    time.sleep(0.5)


def done():
    done = "f"
    done = done.encode()
    blue.write(done)


try:
    while True:
        data = blue.readline()
        # data = data.decode()
        # print(type(data), data)
        # if data != "s":
        #     print("didn't")
        #     continue
        # else:
        print("found s")
        grip(80)
        x, add = search(80, 5)
        drop()
        done()


except KeyboardInterrupt:
    GPIO.cleanup()
    print("Quit")
