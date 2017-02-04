import threading 

class Car(threading.Thread):
    def __init__(self, socket, client_info, client_message):
        threading.Thread.__init__(self)
        self.socket = socket
        self.client_info = client_info
        self.clientmessage = client_message

    def run(self):
        #pass
        a = 1
        #data = self.socket.recv(1024)
        #print "Received %s" , data
        #print ""
        #self.mysend("Message back")

    
    def mysend(self, data):
        self.socket.send(data)
        self.socket.close()
        #print self.client_info, ": %s" % data

        self.close()

    def close (self):
        #self.socket.close()
        self.join()
