import cv2
import numpy as np
import time
import threading
from collections import deque

#import CDC
#x = CDC
#color_detection(x)

class CDC(object):

    intersection_state = [0,0,0,0,0,0,0,0]

    def __init__(self):
        #recieving request 
        color_detect = threading.Thread(target = self.color_detection, args=())
        color_detect.start()

    def color_detection(self):
        
        cap = cv2.VideoCapture(0)
        #time.sleep(5)
        firstFrame = None
        counter = 0
        while (1) :
            time.sleep(0.5)
            _, frame = cap.read()

            #cv2.imshow("frame", frame)

            frame1 = cv2.resize(frame, (800,800))
            fr0 = frame1[0:400, 0:400]
            fr1 = frame1[0:400, 400:800]        
            fr2 = frame1[400:800, 0:400]
            fr3 = frame1[400:800, 400:800]

            hsv0 = cv2.cvtColor(fr0, cv2.COLOR_BGR2HSV)
            hsv1= cv2.cvtColor(fr1, cv2.COLOR_BGR2HSV)
            hsv2 = cv2.cvtColor(fr2, cv2.COLOR_BGR2HSV)
            hsv3 = cv2.cvtColor(fr3, cv2.COLOR_BGR2HSV)
            
            #cv2.imshow("hsv0", hsv0)
            """
            hsv0 = top left
            hsv1 = top right
            hsv2 = bottom left
            hsv3 = bottom right
            """
            
            cv2.imshow("hsv0", hsv0)#RY
            cv2.imshow("hsv1", hsv1)#BG
            cv2.imshow("hsv2", hsv2)#RY
            cv2.imshow("hsv3", hsv3)#BG
            
            cv2.imshow('frame',frame1)
            
            #define range of Red color in HSV
            
            lowerRed = np.array([160,100,100], dtype = np.uint8)
            upperRed = np.array([179,255,255], dtype = np.uint8)

            #define range of yellow color in HSV

            lowerYellow = np.array([15,100,100], dtype = np.uint8)
            upperYellow = np.array([35,255,255], dtype = np.uint8)

            #define range of blue color in HSV

            lowerBlue = np.array([90,100,100], dtype = np.uint8)
            upperBlue = np.array([140,255,255], dtype = np.uint8)

            #define range of Green color in HSV

            lowerGreen = np.array([40,100,100], dtype = np.uint8)
            upperGreen = np.array([80,255,255], dtype = np.uint8)

            """FOR HSV0"""

            # RED mask

            maskRED0 = cv2.inRange(hsv0, lowerRed, upperRed)
            resRED0 = cv2.bitwise_and(fr0,fr0, mask = maskRED0)
            NumRedPixHSV0 = cv2.countNonZero(maskRED0)
            #print 'NumRedPixHSV0 : %d ' %NumRedPixHSV0
            
            #cv2.imshow('maskRED0',maskRED0)
            #cv2.imshow('resRED0',resRED0)

            # YELLOW mask

            maskYELW0 = cv2.inRange(hsv0, lowerYellow, upperYellow)
            resYELW0 = cv2.bitwise_and(fr0, fr0, mask = maskYELW0)
            NumYelwPixHSV0 = cv2.countNonZero(maskYELW0)
            #print 'NumYelwPixHSV0 : %d' %NumYelwPixHSV0
                    
            #cv2.imshow('maskYELW0',maskYELW0)
            #cv2.imshow('resYELW0',resYELW0)

            """FOR HSV3"""

            maskRED3 = cv2.inRange(hsv3, lowerRed, upperRed)
            resRED3 = cv2.bitwise_and(fr3,fr3, mask = maskRED3)
            NumRedPixHSV3 = cv2.countNonZero(maskRED3)
            #print 'NumRedPixHSV3 %d: ' %NumRedPixHSV3
            
            #cv2.imshow('maskRED3',maskRED3)
            #cv2.imshow('resRED3',resRED3)

            # YELLOW mask

            maskYELW3 = cv2.inRange(hsv3, lowerYellow, upperYellow)
            resYELW3 = cv2.bitwise_and(fr3, fr3, mask = maskYELW3)
            NumYelwPixHSV3 = cv2.countNonZero(maskYELW3)
            #print 'NumYelwPixHSV3 : %d ' %NumYelwPixHSV3

            #cv2.imshow('maskYELW3',maskYELW3)
            #cv2.imshow('resYELW3',resYELW3)  

            """ FOR HSV2"""

            # BLUE mask

            maskBLUE2 = cv2.inRange(hsv2, lowerBlue, upperBlue)
            resBLUE2 = cv2.bitwise_and(fr2,fr2, mask = maskBLUE2)
            NumBluePixHSV2 = cv2.countNonZero(maskBLUE2)
            #print 'NumBluePixHSV2 : %d' %NumBluePixHSV2
            
            #cv2.imshow('maskBLUE2',maskBLUE2)
            #cv2.imshow('resBLUE2',resBLUE2)

            # GREEN mask

            maskGREEN2 = cv2.inRange(hsv2, lowerGreen, upperGreen)
            resGREEN2 = cv2.bitwise_and(fr2, fr2, mask = maskGREEN2)
            NumGrnPixHSV2 = cv2.countNonZero(maskGREEN2)
            #print 'NumGrnPixHSV2 : %d'  %NumGrnPixHSV2
                    
            #cv2.imshow('maskGREEN2',maskGREEN2)
            #cv2.imshow('resGREEN2',resGREEN2)

            """FOR HSV1"""

            # BLUE mask

            maskBLUE1 = cv2.inRange(hsv1, lowerBlue, upperBlue)
            resBLUE1 = cv2.bitwise_and(fr1,fr1, mask = maskBLUE1)
            NumBLUEPixHSV1 = cv2.countNonZero(maskBLUE1)
            #print 'NumBLUEPixHSV1 : %d' %NumBLUEPixHSV1
            
            #cv2.imshow('maskBLUE1',maskBLUE1)
            #cv2.imshow('resBLUE1',resBLUE1)

            # GREEN mask

            maskGRN1 = cv2.inRange(hsv1, lowerGreen, upperGreen)
            resGRN1 = cv2.bitwise_and(fr1, fr1, mask = maskGRN1)
            NumGRNPixHSV1 = cv2.countNonZero(maskGRN1)
            #print 'NumGRNPixHSV1 : %d ' %NumGRNPixHSV1  
                    
            #cv2.imshow('maskGRN1',maskGRN1)
            #cv2.imshow('resGRN1',resGRN1)

            last_stateHSV0C = 'Unoccupied'
            last_stateHSV1C = 'Unoccupied'
            last_stateHSV2C = 'Unoccupied'
            last_stateHSV3C = 'Unoccupied'

            last_stateHSV0G = 'Unoccupied'
            last_stateHSV1G = 'Unoccupied'
            last_stateHSV2G = 'Unoccupied'
            last_stateHSV3G = 'Unoccupied'        

        ##        NumRedPixHSV0
        ##        NumYelwPixHSV0
        ##        NumBLUEPixHSV1
        ##        NumGRNPixHSV1
        ##        NumBluePixHSV2
        ##        NumGrnPixHSV2
        ##        NumRedPixHSV3
        ##        NumYelwPixHSV3

            upper_bound = 3000
            lower_bound = 2300
            
        #Incoming positions
            
            # quadrant 0
            
            if (NumYelwPixHSV0 < lower_bound) & (last_stateHSV0C == 'Unoccupied'):
                last_stateHSV0C = 'Occupied'   
                self.intersection_state[4] = 1
            elif NumYelwPixHSV0 > upper_bound:
                last_stateHSV0C = 'Unoccupied'

            # quadrant 1

            if (NumBLUEPixHSV1 < lower_bound) & (last_stateHSV1C == 'Unoccupied'):
                last_stateHSV1C = 'Occupied'   
                self.intersection_state[5] = 1
            elif NumBLUEPixHSV1 > upper_bound:
                last_stateHSV1C = 'Unoccupied'

            # quadrant 2

            if (NumGrnPixHSV2 < lower_bound) & (last_stateHSV2C == 'Unoccupied'):
                last_stateHSV2C = 'Occupied'   
                self.intersection_state[6] = 1
            elif NumGrnPixHSV2 > upper_bound:
                last_stateHSV2C = 'Unoccupied'

            # quadrant 3

            if (NumRedPixHSV3 < lower_bound) & (last_stateHSV3C == 'Unoccupied'):
                last_stateHSV3C = 'Occupied'   
                self.intersection_state[7] = 1
            elif NumRedPixHSV3 > upper_bound:
                last_stateHSV3C = 'Unoccupied'

            # quadrant 1

            if (NumRedPixHSV0 < lower_bound) & (last_stateHSV0G == 'Unoccupied'):
                last_stateHSV0G = 'Occupied'   
                self.intersection_state[0] = 1
            elif NumRedPixHSV0 > upper_bound:
                last_stateHSV0G = 'Unoccupied'

            # quadrant 2

            if (NumGRNPixHSV1 < lower_bound) & (last_stateHSV1G == 'Unoccupied'):
                last_stateHSV1G = 'Occupied'   
                self.intersection_state[1] = 1
            elif NumGRNPixHSV1 > upper_bound:
                last_stateHSV1G = 'Unoccupied'

            #quadrant 3

            if (NumBluePixHSV2 < lower_bound) & (last_stateHSV2G == 'Unoccupied'):
                last_stateHSV2G = 'Occupied'   
                self.intersection_state[2] = 1
            elif NumBluePixHSV2 > upper_bound:
                last_stateHSV2G = 'Unoccupied'

            # quadrant 4

            if (NumYelwPixHSV3 < lower_bound) & (last_stateHSV3G == 'Unoccupied'):
                last_stateHSV3G = 'Occupied'   
                self.intersection_state[3] = 1
            elif NumYelwPixHSV3 > upper_bound:
                last_stateHSV3G = 'Unoccupied'

            

            if counter < 10:
                self.intersection_state = [0,0,0,0,0,0,0,0]
            #print self.intersection_state
                            

            k = cv2.waitKey(5) & 0xFF
        ##        if k == 27:
        ##            break
        ##
        ##    cap.release()
        ##    cv2.destroyAllWindows()
            counter = counter + 1 

    def get_intersection_state(self):
        print 'get_intersection_state called'
        new_intersection_state = self.intersection_state
        self.intersection_state = [0,0,0,0,0,0,0,0]
        
        return new_intersection_state
        

##def main():
####    x = CDC
####    intersection_state = [0,0,0,0,0,0,0,0]
####    color_detection(x,intersection_state)
##
##    x = CDC()
##    CDC.color_detection(x)
##    CDC.get_intersection_state(x)
        
