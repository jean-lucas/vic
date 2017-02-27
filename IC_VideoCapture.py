import numpy as np
import cv2
import imutils
import argparse
import time


#cv2.BackgroundSubtractorMOG2
#cv2.createBackgroundSubtractorGMG

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help = "path to the option video file")
ap.add_argument("-a", "--min-area", type = int, default = 500, help = "minimum area size")
args = vars(ap.parse_args())

if args.get("video", None) is None:
    camera = cv2.VideoCapture(0)
    time.sleep(0.25)
    
firstFrame = None

while (True):
    
    
    (ret, frame) = camera.read()
    text = "Unoccupied"

    if not ret:
        break

    frame = imutils.resize(frame, width=700)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21,21), 0 )

    #cv2.imshow('frame', gray)

    if firstFrame is None:
        firstFrame = gray
        continue

    
    frameDelta = cv2.absdiff(firstFrame, gray)
    thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

    thresh = cv2.dilate(thresh, None, iterations =2)
    ( _, cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        if cv2.contourArea(c) < args["min_area"]:
            continue
        (x,y,w,h) = cv2.boundingRect(c)
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0),2)
        text = "Occupied"

        cv2.putText(frame, "Room Status: {}".format(text), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),2)
        

        cv2.imshow("Security Feed", frame)
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Frame Delta", frameDelta)

            
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows() 
