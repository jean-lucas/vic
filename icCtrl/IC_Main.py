# class dependencies
from TrafficController import TrafficController
from VehicleDetection import VehicleDetection
from Car import Car

# other dependencies 
from collections import deque
import threading
import bluetooth

# these dependances will go... just for testing purposes
import time
import random

# Communcation and main class in the same file
# Having them in separte files wasn't working

# Communication class
class Communication (object):

    bt_port_receive = 1
    proceeds = deque()
    
    def __init__(self):
        # receiveing thread
        recieve_thread = threading.Thread(target=self.recieve, args=())
        recieve_thread.start()

        # sending thread 
        recieve_thread = threading.Thread(target=self.send, args=())
        recieve_thread.start()
    
    def recieve(self):
        time.sleep(0.2)
        print "Bluetooth server has started."
        print
        print "Waiting for connections on RFCOMM channel %d" % self.bt_port_receive
        print
        
        # This is like TCP socket program from 4C03 (sort of)
        server_sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )

        # bind the host address to a particular port
        server_sock.bind(('', self.bt_port_receive))

        # listen on a particular port 
        server_sock.listen(5)

        # in order to use a method of a class need to first create an instance of it
        add_car_to_buffer = IC_Main()
        
        while True:

            client_socket, client_bluetooth_ID = server_sock.accept()

            # particulars to help in the debugging
            print "Connection recieved from: ", client_bluetooth_ID[0],": connection accepted @", time.ctime()
            print

            # recieve the message
            client_message = client_socket.recv(1024)
            client_socket.close()

            # extract the message contents
            current_car = self.message_extraction(client_message, client_bluetooth_ID)
                
            # add car to arrival buffer    
            add_car_to_buffer.add_arrival_queue(current_car)

            # terminate the current request socket
            


    def send(self):
        time.sleep(0.3)
        print "Bluetooth client has started."

        # in order to use a method of a class need to first create an instance of it
        get_car_from_buffer = IC_Main()

        while True:

            if (len(self.proceeds) > 0):
                
##                print "Departure length, %d" % len(self.proceeds)
##                print
                
                car = self.proceeds.pop()
                port = getattr(car, 'port')
                address = getattr(car, 'client_bluetooth_ID')
                message_to_car = getattr(car, 'message_to_car')

                try:
                            
                    send_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)            
                    send_socket.connect((address, int(port)))

                    send_socket.send(message_to_car)
                    send_socket.close()
                except IOError:
                        print "Connection Failed"
                        print
                
            
         
    def message_extraction(self, client_message, client_bluetooth_ID ):

        
        

        # assuming a string for now, elements separated by a comma
        my_content = client_message.split(',')

        
        # message content check (to be implemented)
        

        return Car(my_content[0], client_bluetooth_ID[0], my_content[1], my_content[2], '')

    def add_proceed_queue(self, car):
        self.proceeds.appendleft(car)
        




# Main class
class IC_Main(object):

    # make instances of the other modules so can call their methods
    # probably a different way to do this.... just trying to get this running
    
    traffic = TrafficController()
    detect = VehicleDetection()
    arrivals = deque()

    intersection_cars = [0 for i in range(10)]
    intersection_clear = False

    
    
    

    
    # method to 
    def run(self):

        # this one launches in the background on it own thread
        # and runs till the python shell is restarted 
        '''
        Uncomment to run communication
        communication = Communication()
        '''
        test = 1
        
        
        print "Welcome to VIC! The system has started"
        print

        # main while loop in the IC controller

        while True:
            
            #self.check_intersection_state()
            #self.traffic.test()
            

            if (len(self.arrivals) > 0):
                #print "Arrival length, %d" % len(self.arrivals)
                car = self.arrivals.pop()

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
                
                #time.sleep(.2)
                test=test+1
                print "Message counter %d: " % test
                setattr(car, 'message_to_car',"GO %d" % test)
                #self.add_depart_queue(car)

                '''
                Uncomment to run communication
                communication.add_departure_queue(car)
                '''
           
            #time.sleep(.)
                
        
            


    def remove_queue(self):
        pass

##    def send_response(self, car_response):
##        # gets the object method you wish call and its parameters
##        getattr(car_response, 'mysend')("Message from server: Go")

    def add_proceed_queue(self, car):
        # proceed queue in communications
        communication.add_proceed_queue(car)

    def add_arrival_queue(self,car):
        # arrival queue in IC_Main
        self.arrivals.appendleft(car)


    def check_intersection_state(self):
        print self.detect.update_intersection()
        #this function will retrieve state of intersection and update intersection_cars array for any cars leaving intersection
        




def main():
    a = IC_Main()
    a.run()    

# Allows main to be run directly 
if __name__ == '__main__':
    main()

