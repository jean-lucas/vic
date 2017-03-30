import cv2
import numpy as np
import time

def main():
    cap = cv2.VideoCapture(0)
    firstFrame = None
    while (1) :
        #time.sleep(1)
        ret, frame = cap.read()

        if not ret:
            print "error"

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (41,41), 0 )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if firstFrame is None:
            firstFrame = gray
            

            #define range of Red color in HSV

        lowerRed = np.array([160,100,100], dtype = np.uint8)
        upperRed = np.array([179,255,255], dtype = np.uint8)

        mask = cv2.inRange(hsv, lowerRed, upperRed)
        res = cv2.bitwise_and(frame,frame, mask = mask)
            #cv2.imshow('mask',mask)
            #cv2.imshow('res',res)

        NumRedPix = cv2.countNonZero(mask)

            #define range of blue color in HSV

        lowerBlue = np.array([90,100,100], dtype = np.uint8)
        upperBlue = np.array([140,255,255], dtype = np.uint8)

        mask2 = cv2.inRange(hsv, lowerBlue, upperBlue)
        res2 = cv2.bitwise_and(frame,frame, mask = mask2)
            #cv2.imshow('mask2',mask2)
            #cv2.imshow('res2',res2)

        NumBluePix = cv2.countNonZero(mask2)

            #define range of yellow color in HSV

        lowerYellow = np.array([20,100,100], dtype = np.uint8)
        upperYellow = np.array([30,255,255], dtype = np.uint8)

        mask3 = cv2.inRange(hsv, lowerYellow, upperYellow)
        res3 = cv2.bitwise_and(frame,frame, mask = mask3)
            #cv2.imshow('mask3',mask3)
            #cv2.imshow('res3',res3)

        NumYelwPix = cv2.countNonZero(mask3)

            #define range of Green color in HSV

        lowerGreen = np.array([40,100,100], dtype = np.uint8)
        upperGreen = np.array([80,255,255], dtype = np.uint8)

        mask1 = cv2.inRange(hsv, lowerGreen, upperGreen)
        res1 = cv2.bitwise_and(frame,frame, mask = mask1)
            #cv2.imshow('mask1',mask1)
            #cv2.imshow('res1',res1)

        NumGrnPix = cv2.countNonZero(mask1)
        cv2.imshow('frame',frame)

        print NumRedPix, NumYelwPix, NumBluePix, NumGrnPix   

        #k = cv2.waitKey(5) & 0xFF
        #if k == 27:
        #    break
