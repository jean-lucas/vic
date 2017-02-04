# class dependencies
from TrafficController import TrafficController
from VehicleDetection import VehicleDetection
from Car import Car

# other dependencies 
from collections import deque
import threading
#import bluetooth

# these dependances will go... just for testing purposes
import time
import random

# Communcation and main class in the same file
# Having them in separte files wasn't working

# Communication class
class Communication (object):

    bt_port = 2
    
    def __init__(self):
        # runs the communcation in the background
        thread = threading.Thread(target=self.run, args=())\

        # this allows communication to keep running even if main
        # thread crashes 
        thread.daemon = True                            
        thread.start()
 


    
    def run(self):
        time.sleep(0.3)
        print "Bluetooth server has started."
        print
        print "Waiting for connections on RFCOMM channel %d" % self.bt_port

        # This is like TCP socket program from 4C03 (sort of)
        server_sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )

        # bind the host address to a particular port
        server_sock.bind(('', self.bt_port))

        # listen on a particular port 
        server_sock.listen(self.bt_port)

        # in order to use a method of a class need to first create an instance of it
        add_car_to_buffer = IC_Main()
        
        while True:

            client_socket, client_info = server_sock.accept()

            # particulars to help in the debugging
            print "Client: ", client_info, ": connection accepted @", time.ctime()
            print

            client_message = client_socket.recv(1024)

            # create a car object and keep the bluetooth connect live on a thread
            current_car = Car(client_socket, client_info, client_message)
            current_car.setDaemon(True)
            current_car.start()        
    
    
            add_car_to_buffer.add_arrival_queue(current_car)
            
            





# Main class
class IC_Main(object):

    # make instances of the other modules so can call their methods
    # probably a different way to do this.... just trying to get this running
    
    traffic = TrafficController()
    detect = VehicleDetection()
    arrivals = deque()

    # this one launches in the background on it own thread
    # and runs till the python shell is restarted 
    #communication = Communication()
    

    
    # method to 
    def run(self):
        print "Welcome to VIC! The system has started"
        print

        # main while loop in the IC controller

        while True:
            
            self.check_intersection_state()
            self.traffic.test()
            print

            if (len(self.arrivals) > 0):

                
                
                car = self.arrivals.pop()
                self.send_response(car)
                
                

            time.sleep(3)
                
        
            


    def remove_queue(self):
        pass

    def send_response(self, car_response):
        # gets the object method you wish call and its parameters
        getattr(car_response, 'mysend')("Message from server: Go")

    def add_arrival_queue(self,car):
        self.arrivals.appendleft(car)


    def check_intersection_state(self):
        print self.detect.update_intersection()
        




def main():
    a = IC_Main()
    a.run()    

# Allows main to be run directly 
if __name__ == '__main__':
    main()

