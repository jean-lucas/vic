class Car(object):
    def __init__(self, car_ID, port, client_bluetooth_ID, direction_from, direction_to, time_stamp):
    	
    	self.car_ID = car_ID # unique vehicle ID

        self.port = port # vehicle listening port
        self.client_bluetooth_ID = client_bluetooth_ID # vehicle Bluetooth address 

        self.direction_from = direction_from 
        self.direction_to = direction_to

        self.time_stamp = time_stamp 

        self.vehicle_command = 0 # various vehicle command codes

        self.proceed_now = False
        self.retransmission_number = 0 
