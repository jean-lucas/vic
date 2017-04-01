# external class dependencies
from ColorDetection import ColorDetection
from Car import Car
from Communication import Communication



# Dependancies for debugging
import time
import random

# Main class
class IC_Main(object):

    communication = Communication()

    intersection_cars = [0 for i in range(4)]
    intersection_clear = False
    car = -1

    # for testing purposes
    test = 0
    
    # method to 
    def run(self):

        print "Welcome! VIC system has started."

        # main while loop in the IC controller

        while True:
            
            #self.update_intersection_state()
            #self.traffic.test()
            
            
            message_check = self.communication.arrival_check()
            
            if ((message_check == 1)):
                
                self.car = self.communication.arrival_deque()

                #print "Car direction_from: %s" % (getattr(self.car,'direction_from'))
               
                #print "Message count: %d. " % test
                self.test+=1
                #setattr(car, 'message_to_car',"GO %d" % test)

                #time.sleep(0.3)
                #start = 0
                #current_car_index = -1

                for i in range(len(self.intersection_cars)):
                    if(self.intersection_cars[i]==0):
                        self.intersection_cars[i] = self.car
                        current_car_index = i
                        break

                while (getattr(self.intersection_cars[current_car_index], 'proceed_now') == False):
                    self.check_intersection_state(current_car_index)
                   # print self.intersection_clear

                    if(self.intersection_clear):
                        setattr(self.intersection_cars[current_car_index],'proceed_now',True)
                    else:
                        for i in range(len(self.intersection_cars)):
                            if(self.intersection_cars[i]!=0 and i!=current_car_index):
                                if(getattr(self.intersection_cars[current_car_index], 'direction_from')==getattr(self.intersection_cars[i], 'direction_from')): #or getattr(self.intersection_cars[current_car_index], 'direction_from')==getattr(self.intersection_cars[i], 'direction_to')):
                                    setattr(self.intersection_cars[current_car_index],'proceed_now',True)
                                    #break   #this break only works if we have 2 cars on the track (more efficient)
                                else:
                                    setattr(self.intersection_cars[current_car_index],'proceed_now',False)
                    if(getattr(self.intersection_cars[current_car_index],'proceed_now')):
                        print "Safe passage granted"
                    else:
                        print "Safe passage NOT granted"


                #current car's proceed_now is true, then add car to communication's proceed queue

                self.communication.proceed_enqueue(self.car)
                print "New car added"
                
        

    #def arrival_enqueue(self,car):
        # arrival queue in IC_Main
        #self.arrivals.appendleft(car)


    def check_intersection_state(self,current_car_index):
        #update intersection_cars array from camera info for cars leaving

        #check if intersection is empty
        for i in range(len(self.intersection_cars)):
            if(i!=current_car_index and self.intersection_cars[i]!=0):
                self.intersection_clear = False
                break
            else:
                self.intersection_clear = True
        #print self.detect.update_intersection()
        #this function will retrieve state of intersection and update intersection_cars array for any cars leaving intersection
        


def main():
    a = IC_Main()
    #x = ColorDetection()
    #while True:
     #   x.getIntersectionState()
    a.run()    

# Allows main to be run directly 
if __name__ == '__main__':
    main()

