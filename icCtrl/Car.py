class Car(object):
    def __init__(self, port, client_bluetooth_ID, direction_from, direction_to, message_to_car):
        self.port = port
        self.client_bluetooth_ID = client_bluetooth_ID
        self.direction_from = direction_from
        self.direction_to = direction_to
        self.message_to_car = message_to_car
        self.proceed_now = False
