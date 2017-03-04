# external class dependencies
from TrafficController import TrafficController
from VehicleDetection import VehicleDetection
from Car import Car

# module depenencies 
from collections import deque
import threading

# pyBluez Bluetooth library 
import bluetooth

# Dependancies for debugging
import time
import random



class Communication (object):
   
    '''
        Communication module for VIC's intersction controller. The communication medium
        is Bluetooth. This module requires pyBluez Bluetooth library to be installed on
        the host machine. 

        The module contains two main parts, a thread to listen for vehicle requests, 
        and a thread to send vehicle requests. Car objects are created for each vehicle 
        request and enqueued to an arrival queue in the IC_Main class. Proceed commands are 
        extracted from Car objects dequeued from a proceed queue located within the 
        Communication class. 

        The listening port for the bluetooth recieve server is port 1.

        The destination Bluetooth address and ports are retrieved from vehicle requests. 

    ''' 


    # bluetooth recieve listening port
    RECIEVE_PORT = 1

    # proceed message queues
    proceeds = deque()
    
    def __init__(self):

        # receiving thread 
        recieve_requests = threading.Thread(target=self.recieve, args=())
        recieve_requests.start()

        # sending thread 
        send_commands = threading.Thread(target=self.send, args=())
        send_commands.start()
    
    def recieve(self):
        time.sleep(0.2)
        
        print "Bluetooth server waiting for connections on RFCOMM channel %d." % self.RECIEVE_PORT
        print
        
        # Bluetooth socket using RFCOMM communication protocol 
        server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )

        # bind the host address to a particular port
        server_sock.bind(('', self.RECIEVE_PORT))

        # internal bluetooth buffer size
        server_sock.listen(5)

        # instantiate arrival queue 
        vehicle_arrivals = IC_Main()
        
        while True:

            # accept a connecting socket
            client_socket, client_bluetooth_ID = server_sock.accept()

            # particulars to help in the debugging
            print "Connection recieved from: ", client_bluetooth_ID[0],": connection accepted @", time.ctime()
            print

            # recieve vehicle request
            client_message = client_socket.recv(1024)
            client_socket.close()

            # extract vehicle request contents
            current_car = self.message_extraction(client_message, client_bluetooth_ID)
                
            # enqueue arrival buffer    
            vehicle_arrivals.arrival_enqueue(current_car)

            client_socket.close()
            


    def send(self):
        time.sleep(0.3)
        print "Bluetooth client has started."

        while True:

            if (len(self.proceeds) > 0):
                
            
                car = self.proceeds.pop()
                port = getattr(car, 'port')
                address = getattr(car, 'client_bluetooth_ID')
                #print "Preparing to send to %s" % address
                message_to_car = getattr(car, 'message_to_car')

                try:
                    #for testing purposes
                    #time.sleep(1)
                    send_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)            
                    send_socket.connect((address, int(port)))

                    send_socket.send(message_to_car)
                    send_socket.close()

                except IOError:
                    if (getattr(car, 'retransmission_number') < 3):
                        test = getattr(car, 'retransmission_number')
                        setattr(car, 'retransmission_number', test + 1)
                        self.proceeds.append(car)
                        print "Connection Failed, retransmission being attempted."
                    else:
                        print "Connection Failed, message dropped."
                        print

                        
                
            
         
    def message_extraction(self, client_message, client_bluetooth_ID ):

        # assuming a string for now, elements separated by a comma
        my_content = client_message.split(',')

        
        # message content check (to be implemented)
        

        return Car(my_content[0], client_bluetooth_ID[0], my_content[1], my_content[2], '')

    def proceed_enqueue(self, car):
        self.proceeds.appendleft(car)
        




# Main class
class IC_Main(object):

    traffic = TrafficController()
    detect = VehicleDetection()
    arrivals = deque()

    intersection_cars = [0 for i in range(10)]
    intersection_clear = False

    #communication = Communication()
    
    

    
    # method to 
    def run(self):

        
        
        test = 1
        
       
        print "Welcome! VIC system has started."
        print

        # main while loop in the IC controller

        while True:
            
            #self.check_intersection_state()
            #self.traffic.test()
            

            if (len(self.arrivals) > 0):
                print "Arrival length, %d" % len(self.arrivals)
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
                self.proceed_enqueue(car)


                
##                test=test+1
##                print "Message count: %d. " % test
##                setattr(car, 'message_to_car',"GO %d" % test)
##                #time.sleep(.01)
                

             
                
        
        
    def proceed_enqueue(self, car):
        # proceed queue in communications
        self.communication.proceed_enqueue(car)

    def arrival_enqueue(self,car):
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

