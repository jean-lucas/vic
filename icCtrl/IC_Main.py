# external class dependencies
from TrafficController import TrafficController
from VehicleDetection import VehicleDetection
from Car import Car
from Communication import Communication 




# Dependancies for debugging
import time
import random

# Main class
class IC_Main(object):

    
    

    
    # method to 
    def run(self):


        traffic = TrafficController()
        detect = VehicleDetection()
    

        intersection_cars = [0 for i in range(10)]
        intersection_clear = False

        # for testing purposes
        test = 0
        communication = Communication()

        
        car = -1



        print "Welcome! VIC system has started."
        print
        
      
        
        #car = communication.arrival_deque()

        
        

        # main while loop in the IC controller

        while True:
            
            #self.check_intersection_state()
            #self.traffic.test()
            
            # MATT IF STATEMENT
            
            #if (self.arrivals) != -1):
                #print "Arrival length, %d" % len(self.arrivals)
                #car = self.arrivals.pop()

            
            
            message_check = communication.arrival_check()
            
            if ((message_check == 1)):
                
                car = communication.arrival_deque()
               
                #print "Message count: %d. " % test
                test+=1
                #setattr(car, 'message_to_car',"GO %d" % test)
                time.sleep(1)
                start = 0
                current_car_index = -1

                 for i in range(len(self.intersection_cars)):
                      if(self.intersection_cars[i]==0):
                          self.intersection_cars[i] = car
                          current_car_index = i
                          break

                 while (getattr(self.intersection_cars[current_car_index], 'proceed_now') == False):
                      check_intersection_state()

                      if(intersection_clear==True):
                          setattr(self.intersection_cars[current_car_index],'proceed_now',True)
                      else:
                          for i in range(len(self.intersection_cars)):
                              if(self.intersection_cars[i]!=0 and i!=current_car_index):
                                  if(getattr(self.intersection_cars[current_car_index], 'direction_from')==getattr(self.intersection_cars[i], 'direction_from') or getattr(self.intersection_cars[current_car_index], 'direction_from')==getattr(self.intersection_cars[i], 'direction_to')):
                                      setattr(self.intersection_cars[current_car_index],'proceed_now',True)
                                      break   #this break only works if we have 2 cars on the track (more efficient)

                #TODO here: if current car's proceed_now is true, then tell communication to send car a 'GO' message
                communication.proceed_enqueue(car)
               


                
               
                

             
                
        

    #def arrival_enqueue(self,car):
        # arrival queue in IC_Main
        #self.arrivals.appendleft(car)


    def check_intersection_state(self):
        print self.detect.update_intersection()
        #this function will retrieve state of intersection and update intersection_cars array for any cars leaving intersection
        




def main():
    a = IC_Main()
    a.run()    

# Allows main to be run directly 
if __name__ == '__main__':
    main()

