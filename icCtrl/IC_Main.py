# external class dependencies
#from ColorDetection import ColorDetection
from Car import Car
from Communication import Communication
from VehicleDetection import VehicleDetection
#from CDC import CDC 



# Dependancies for debugging
import time
import random

# Main class
class IC_Main(object):

    communication = Communication()
    #cdc = CDC()

    intersection_cars = [0 for i in range(2)]
    intersection_clear = False
    car = -1

    # for testing purposes
    test = 0

    count = 0
    
    # method to 
    def run(self):

        print "Welcome! VIC system has started."

        leflag = 0

        # main while loop in the IC controller

        while True:
            
            #self.update_intersection_state()
            #self.traffic.test()
            
            message_check = self.communication.arrival_check()
            #print "message check %d" % (message_check)
            if (message_check==1):
                
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

                while (self.intersection_cars[current_car_index].proceed_now == False):
                    self.check_intersection_state(current_car_index)
                   # print self.intersection_clear

                    if(self.intersection_clear):
                        self.intersection_cars[current_car_index].proceed_now=True
                    else:
                        for i in range(len(self.intersection_cars)):
                            if(self.intersection_cars[i]!=0 and i!=current_car_index):
                                if(self.intersection_cars[current_car_index].direction_from==self.intersection_cars[i].direction_from or self.intersection_cars[current_car_index].direction_from==self.intersection_cars[i].direction_to or self.intersection_cars[current_car_index].direction_to==self.intersection_cars[i].direction_from):
                                    self.intersection_cars[current_car_index].proceed_now=True
                                    #break
                                else:
                                    self.intersection_cars[current_car_index].proceed_now=False
                    if(self.intersection_cars[current_car_index].proceed_now):
                        print "Safe passage granted"
                        leflag = 0
                    else:
                        if(leflag == 0):
                            print "Safe passage NOT granted"
                            leflag = 1



                #current car's proceed_now is true, then add car to communication's proceed queue

                self.communication.proceed_enqueue(self.car)
                print "New car added"
            #print "---------------"
            #print "About to call cars leave method"
            #print "---------------"
            self.check_intersection_state(-1)

            #time.sleep(1.5)
            # if(cars_leaving[1] != 0):
            #     print "shouldnt be here"
            #     for i in range(len(self.intersection_cars)):
            #         if(self.intersection_cars[i]!=0):
            #             if(cars_leaving[1] == self.intersection_cars[i].car_id):
            #                 self.intersection_cars[i] = 0;
            #                 print "Car left intersection"        
        

    #def arrival_enqueue(self,car):
        # arrival queue in IC_Main
        #self.arrivals.appendleft(car)


    def check_intersection_state(self,current_car_index):
        cars_leaving = self.communication.get_cars_leaving()
        #print "car leaving: %d" % (cars_leaving)
        
        if(cars_leaving != 0):
            #print "car leaving 0 %d and 1 %d" % (cars_leaving[0],cars_leaving[1])
            #print "Car leaving: %d" % (cars_leaving)
            print "in cars_leaving block"
            print cars_leaving
            for i in range(len(self.intersection_cars)):
                if(self.intersection_cars[i]!=0):
                    print "-------------------------------"
                    print self.intersection_cars[i].car_id
                    if(cars_leaving == self.intersection_cars[i].car_id):
                        self.intersection_cars[i] = 0;
                        cars_leaving = 0
                        print "Car left intersection"


                            
        #check if intersection is empty
        if(current_car_index != -1):
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

