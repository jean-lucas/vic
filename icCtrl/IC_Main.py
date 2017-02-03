
from TrafficController import TrafficController
from VehicleDetection import VehicleDetection
from Communication import Communication




import time
import random




# Main class
class IC_Main(object):

    # make instances of the other modules so can call their methods
    # probably a different way to do this.... just trying to get this running
    
    traffic = TrafficController()
    detect = VehicleDetection()

    # this one launches in the background on it own thread
    # and runs till the python shell is restarted 
    communication = Communication()
    

    
    # method to 
    def run(self):
        print "Welcome to VIC! The system has started"
        self.check_intersection_state()
        self.traffic.test()
     
        


    def remove_queue(self):
        pass

    def add_queue(self):
        pass

    def check_intersection_state(self):
        print self.detect.update_intersection()
        
       
        



# we need main mondule (not a main class)
def main():
    a = IC_Main()
    a.run()    

# Allows main to be run directly 
if __name__ == '__main__':
    main()

