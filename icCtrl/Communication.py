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
    RECIEVE_PORT = 8

    arrivals = deque() # arrival message queue
    proceeds = deque() # proceed message queue

    car_to_remove = 0
   
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

            # particulars to help in the debugging ---------------------------------
            print "Connection recieved from: ", client_bluetooth_ID[0],": connection accepted @", time.ctime()
            client_message = client_socket.recv(1024)

            print "Message number %d" % counter
            print " Client Message: ", client_message
            counter +=1
            print
            # ----------------------------------------------------------------------

            # recieve vehicle request
            
            
            #print "The client message: " + client_message

            # extract vehicle request contents (by calling extract method)
            self.message_extraction(client_message, client_bluetooth_ID)
                
            # enqueue arrival buffer    
            # vehicle_arrivals.arrival_enqueue(current_car)
            # --------------------------------------------------------------------------------------

            client_socket.close()
            

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
                vehicle_command = getattr(car, 'vehicle_command')
                
                # for testing purposes 
                ''''
                vehicle_command = str(counter); # check this is correct
                counter += 1;
                '''


                try:
                    #for testing purposes
                    #time.sleep(1)
                    send_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)            
                    send_socket.connect((address, int(port)))

                    send_socket.send(str(vehicle_command))
                    send_socket.close()
                    print "---------------------"
                    print "Proceed signal sent!"
                    print "---------------------"

                except IOError:
                    if (getattr(car, 'retransmission_number') < 3):
                        test = getattr(car, 'retransmission_number')
                        setattr(car, 'retransmission_number', test + 1)
                        self.proceeds.append(car) # append car object to top of queue to attempt a retransmit 
                        print "Connection Failed, retransmission being attempted."
                        print

                        time.sleep(0.5) # try again in 0.3 seconds
                        
                    else:
                        print "Connection Failed, message dropped."
                        print

                        
                
            
         
    def message_extraction(self, client_message, client_bluetooth_ID ):

        '''
        Colour to integer mapping:
        Coming from: >> Going to
            R --> 0  >>  B --> 4
            B --> 1  >>  G --> 5
            G --> 2  >>  Y --> 6
            Y --> 3  >>  R --> 7
        '''

        
        # message deliminator "_"
        # "carID_comingFrom_listenPort_timestamp"
        try:
            my_content = client_message.split('_')
            my_content[4] = int(my_content[4])
            #print my_content
            #my_content[0] is the message type: 0 for entering intersection, 0 for exiting
            if(int(my_content[0]) == 0):

                my_content[1] = int(my_content[1])
                my_content[2] = int(my_content[2])
                
                direction_to = -1

                if (my_content[1] == 0):
                    if (my_content[2] == 0):
                        direction_to = 3
                    else:
                        direction_to = 1
                elif (my_content[1] == 1):
                    if (my_content[2] == 0):
                        direction_to = 2
                    else:
                        direction_to = 0
                elif (my_content[1] == 2):
                    if (my_content[2] == 0):
                        direction_to = 1
                    else:
                        direction_to = 3
                elif (my_content[1] == 3):
                    if (my_content[2] == 0):
                        direction_to = 0
                    else:
                        direction_to = 1
                
                # Car(self, msg_type, port, client_bluetooth_ID, direction_from, direction_to, car_id)
                print my_content[0],"_", my_content[3], "_", client_bluetooth_ID[0], "_", my_content[1], "_", direction_to, "_", my_content[4]
                new_car = Car(my_content[0], my_content[3], client_bluetooth_ID[0], my_content[1], direction_to, my_content[4])
                # add vehicles to the arrival buffer
                self.arrivals.appendleft(new_car)
                print len(self.arrivals)
            else:
                print "car leaving"
                print my_content[4]
                #update car to remove from ic array
               # if (self.car_to_remove[0] == 0):
                self.car_to_remove = my_content[4]
                print self.car_to_remove
                #else:
                 #   self.car_to_remove[1] = my_content[4]
                  #  print self.car_to_remove
        except Exception as e: 
            print(e)
            pass


    def arrival_check(self):
        if (len(self.arrivals)>0):
            return 1
        else:
            return -1

    # called from IC_Main (retrieves arriving car request)
    def arrival_deque(self):

        if (len(self.arrivals)>0):
            return self.arrivals.pop()
        else:
            return -1

    def get_cars_leaving(self):
        cars_leaving = self.car_to_remove
        self.car_to_remove = 0
        if(cars_leaving!=0):
            print "cars_leaving isnt 0"
        #self.car_to_remove[1] = 0
        return cars_leaving


    # called from IC_Main (appends car proceed response)
    def proceed_enqueue(self, car):
        self.proceeds.appendleft(car)

    
