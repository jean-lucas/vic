"""
A simple Python script to send messages to a sever over Bluetooth
using PyBluez (with Python 2).
"""

import bluetooth
from random import randint
import threading
import time


class Communication(object):
    #serverMACAddress = 'CC:AF:78:EB:E8:A8' # change this to Radhika's 
    #serverMACAddress = '14:2D:27:CE:EF:1C'
    serverMACAddress = '34:E6:AD:8B:B2:20' # Matt's address
    #serverMACAddress = 'E8:B1:FC:F7:5D:9F' # Radhika's address
    
    port_send = 8
    port_recieve = 3
    size = 1024

    # for reading in vehicles
    vehicle_requests = []

    # for taking keyboard input
    vehicle_single_request = ""

    # don't send/ask for next request till get response back
    vehicle_command = True


    def __init__(self):
        '''
        Car data reads car requests in from a file
        and store the requests in vehicle_requests array
        '''
        #self.carData()

        '''
        Reads in keyboard input and stores the request in
        vehicle_single_request
        '''
        #
        
        # receiving thread 
        recieve_requests = threading.Thread(target=self.recieve, args=())
        recieve_requests.start()

        # sending thread 
        send_commands = threading.Thread(target=self.send, args=())
        send_commands.start()

        
    
    def send(self):

        
        counter = 0;
        print "Client started"
        
        '''
            requests from file 
        ''' 
        #while (counter < len(self.vehicle_requests)):
        
        '''
            keyboard input requests 
        '''                
            
        while (True):
            
            try:
                
                

                # send next message after random amount of time after a response has been generated
                
                time.sleep(1)
                if (self.vehicle_command == True):

                    '''
                        requests from file
                    '''
                    #send_message = self.vehicle_requests[counter]

                    self.carSingle()

                    s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                    s.connect((self.serverMACAddress, self.port_send))
                    

                    '''
                        keyboard input requests 
                    '''                    
                    send_message = self.vehicle_single_request
                    #print send_message
                
                    
                    #send_message = "1,2,3,4"
                    print "Client sending: "+ send_message
                    print
                    s.send(send_message)

                    s.close()

                    self.vehicle_command = False

                    #time.sleep(1)

                    

                
                    
                #

                # simulate a car going around the track
                #random_sleep_time = randint(2,4)
                
                #print "Sleeping for ", random_sleep_time
                #time.sleep(random_sleep_time)
                
                counter += 1
                
                
            except IOError:
                print "Connection Failed"
                print

                #random_sleep_time = randint(1,4)
                #random_sleep_time = 5

                # restansmitt if the connect has failed (don't do it to fast, 
                    # try to let the connectin stabilize)
                #random_sleep_time = randint(0.3)
                time.sleep(1)
                      
                #print "Sleeping for ", random_sleep_time

                #time.sleep(random_sleep_time)
                
            

    def recieve(self):
       
        
        counter = 0;
        
        s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        s.bind(('', self.port_recieve))
        s.listen(5)

        #time.sleep(.3)

        print "Server has started!"

        while True:
            #try:
            client, clientInfo = s.accept()
            data = client.recv(self.size)

            

            counter  += 1
            print   "Message: " + data
            print

            self.vehicle_command = True
            
            
            client.close()


    def carData(self):
    
        for line in open('C:/Python_27/SE4G06-Code/Capstone_IC_Controller/car_data.csv', 'r'):
            temp = line.split(',')
            #print line
            self.vehicle_requests.append(temp[0] + '_' + temp[1] + '_' + temp[2] + '_' + temp[3] + '_' + temp[4])

              
        #for x in self.vehicle_requests: 
            #print x

    '''
        Allows the user to input a vehicle request by hand
        and creates the request
    '''
    def carSingle(self):
        request_input = raw_input("carID  comingFrom  listeningPort(3)  timestamp (put commas and no space) \n")
        temp = request_input.split(',')
        try:
            self.vehicle_single_request = temp[0] + '_' + temp[1] + '_' + temp[2] + '_' + temp[3] + '_' + temp[4]
        except:
            pass
                
class Main(object):
    def run(self):
        communication = Communication()

def main():
    a = Main()
    a.run()    
    

# Allows main to be run directly 
if __name__ == '__main__':
    main()





    
