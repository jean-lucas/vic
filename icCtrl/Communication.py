# module depenencies 
from collections import deque
from Car import Car

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

    arrivals = deque() # arrival message queue
    proceeds = deque() # proceed message queue
   
    '''
    Send and receive are run on separate threads. This allows sending and 
    recieveing to happend simultaneously. In addition, it also allows
    a different technology to be used for the send and receive. 
    '''
    def __init__(self):

        # receiving thread 
        recieve_requests = threading.Thread(target=self.recieve, args=())
        recieve_requests.start()

        # sending thread 
        send_commands = threading.Thread(target=self.send, args=())
        send_commands.start()
    
    # recieves all vehicle requests and puts them in arrival queue
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

        # instantiate arrival queue (unnecessary as arrival queue has been moved to communication class) --------------------------------------------------
        # vehicle_arrivals = IC_Main()

        counter = 0;
        
        while True:

            # accept a connecting socket
            client_socket, client_bluetooth_ID = server_sock.accept()

            # particulars to help in the debugging
            print "Connection recieved from: ", client_bluetooth_ID[0],": connection accepted @", time.ctime()
            print "Message number %d" % counter
            counter +=1
            print

            # recieve vehicle request
            client_message = client_socket.recv(1024)
            client_socket.close()

            # extract vehicle request contents (by calling extract method)
            current_car = self.message_extraction(client_message, client_bluetooth_ID)
                
            # enqueue arrival buffer    
            # vehicle_arrivals.arrival_enqueue(current_car)
            # --------------------------------------------------------------------------------------

            # add vehicles to the arrival buffer
            self.arrivals.appendleft(current_car)
            


            #client_socket.close()
            

    '''
    Sends all proceed requests in the proceed queue. If a connection fails
    the sender attempts to retransmit to a maximum of three times before the
    message is dropped. 
    '''
    def send(self):

        counter = 0;
        time.sleep(0.3) # For testing purposes only. To be removed.


        print "Bluetooth client has started."

        while True:

            # only attempt to send messages if they are available 
            if (len(self.proceeds) > 0):
                
                
                
                car = self.proceeds.pop() # retrive the car object 
                port = getattr(car, 'port') # retrive the port number
                address = getattr(car, 'client_bluetooth_ID') # retrive the destination address
                # print "Preparing to send to %s" % address
                # message_to_car = getattr(car, 'message_to_car')
                
                message_to_car = str(counter); # check this is correct
                counter += 1;

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
                        self.proceeds.append(car) # append car object to top of queue to attempt a retransmit 
                        print "Connection Failed, retransmission being attempted."
                        print

                        time.sleep(0.3) # try again in half a second
                        
                    else:
                        print "Connection Failed, message dropped."
                        print

                        
                
            
         
    def message_extraction(self, client_message, client_bluetooth_ID ):

        # assuming a string for now, elements separated by a comma
        my_content = client_message.split(',')

        
        # message content check (to be implemented)
        

        return Car(my_content[0], client_bluetooth_ID[0], my_content[1], my_content[2])

    def arrival_check(self):
        if (len(self.arrivals)>0):
            return 1
        else:
            return -1

    # called from IC_Main (retrieves arriving car requests)
    def arrival_deque(self):

        if (len(self.arrivals)>0):
            return self.arrivals.pop()
        else:
            return -1


    # called from IC_Main (appends car proceed response)
    def proceed_enqueue(self, car):
        self.proceeds.appendleft(car)

    
