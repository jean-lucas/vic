# this class contains all the vehicle detection information


class VehicleDetection(object):

    CAMERA_ID = 1234

    # this is dynamic in size 
    intersection_state = [0, 0, 0 ,0 ,0, 0 ,0 ,0 ,0]


    def update_intersection(self):

        # need the self when calling global class variables
        self.intersection_state[0] = 1
        self.intersection_state[1] = 2
        self.intersection_state[2] = 3
        return self.intersection_state
    
