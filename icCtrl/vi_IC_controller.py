# imports
import bluetooth
import threading
from threading import Thread
import time
from collections import deque

# global variables 
hostAddress = 'CC:AF:78:EB:E8:A8'
arrivals = deque()
removals = deque()
END = ""
bt_port = 1

message_counter = 1000

class car(threading.Thread):
    def __init__(self, socket, client_info, client_message):
        threading.Thread.__init__(self)
        self.socket = socket
        self.client_info = client_info
        self.clientmessage = client_message

    def run(self):
        data = self.socket.recv(1024)
        print "Received %s" , data
    
    def mysend(self, data):
        self.socket.send(data)
        print self.client_info, ": %s" % data

    def close (self):
        self.socket.close()






# intersection state method (u)
def intersection_state():
   # print "Intersection State was reached "
   time.sleep(0.4)

def proceed_request():
    #print "Intersection Proceed request"

    #simulate a decision
    time.sleep(.5)
    
    



# communication method
# only receives the requests 
def bt_server():

    client_message = ""
    print "Server Reached"



    # This is like TCP socket program from 4C03 (sort of)
    server_sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
    
    # bind the host address to a particular port
    server_sock.bind((hostAddress, bt_port))

    # listen on a particular port 
    server_sock.listen(bt_port)

    
    print "Waiting for connections on RFCOMM channel %d" % bt_port

    
    global message_counter

    while client_message != "end":
        client_socket, client_info = server_sock.accept()

        # particulars to help in the debugging
        print "Client: ", client_info, ": connection accepted @", time.ctime()
        print

        # client_message = client_socket.recv(1024)
        # printing the first message here
       # print client_message
       # print

        # create a car object
        current_car = car(client_socket, client_info, client_message)
        current_car.setDaemon(True)

        current_car.start()

        
        #echo.setDaemon(True)
        #echo.start()
        arrivals.appendleft(current_car)
        #print "Arrival queue lenght is:", len(arrivals) 
       


        
        if client_message == "end":
            END = "END"
        
        
       

        #client_socket.send("Message 1")
       
        
        # this doesn't close the listening port (as explain in 4C03)
        # this just allows resources to be freed up
        #client_socket.close()

# run method

def run():

    car_request = ""

    # get vehicle from incoming vehile requests buffer
    # only take something if an element is there
    if len(arrivals) > 0:
        car_request = arrivals.pop()

    # simulate getting an intersection state
    intersection_state()

    # simulate a proceed request
    # always assuming true
    proceed_request()

    # then send a response back from here?

    global message_counter 
    message_counter += 2

    if car_request != "":
        test = "Message from server"
        ## this is where we send the message
        car_request.mysend(test)
        car_request.close()
        car_request = ""
    
    #print
    #print time.ctime()

    #print "Sleeping for a couple secs..."

    # just so things don't get out of hand
    time.sleep(.3)

   
    
    #print "I'm running"

    


# main method
if __name__ == '__main__':


    # get the bluetooth server up and running on separate thread
    bt_server = Thread (target = bt_server)
    bt_server.setDaemon(True)
    
    try:
        bt_server.start()
    except :
        
        
    time.sleep(0.3)
    
    
    while END != "END":
        run()


    # potentially for stopping... --> probably don't need this
    # this part of the code will never be reached
    bt_server.join()
    print "Program terminated"

























