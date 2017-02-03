import time
import threading
import random

class Communication (object):

    def __init__(self):
        """ Constructor
        :type interval: int
        :param interval: Check interval, in seconds
        """
        

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()
 


    
    def run(self):
        print """ Method that runs forever """
        while True:
            # Do something
            #print('Doing something imporant in the background')
            self.printTest()

        

    
    def printTest(self):
        a = random.randint(0,6)
        time.sleep(a)
        print "Hello from communication. I'm running in the backround."
        print "I just slept for %ds" %  a



    def returnCar(self):
        return "Here is a car"
