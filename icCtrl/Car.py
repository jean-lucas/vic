class Car(object):
    def __init__(self, msg_type, port, client_bluetooth_ID, direction_from, direction_to, car_id):
    	
    	self.msg_type = msg_type # unique vehicle ID

        self.port = port # vehicle listening port
        self.client_bluetooth_ID = client_bluetooth_ID # vehicle Bluetooth address 

        self.direction_from = direction_from 
        self.direction_to = direction_to

        self.car_id = car_id

        self.vehicle_command = 0 # various vehicle command codes

        self.proceed_now = False
        self.retransmission_number = 0
