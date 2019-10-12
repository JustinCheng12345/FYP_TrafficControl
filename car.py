from utility import *

class LiftCar:
    def __init__(self, car_id, position):
        #self.jobQueue = []
        self.ID = car_id
        self.position = position
        self.status = 0 # -1 Going down/0 Stay/1 Going up/9 Loading
        self.blockQueue = []
        self.pickupQueue = []
        self.dropOffQueue = []
        self.time_multiplier = [1, 1] # multiplier for distance calculation for travel time and stop time

    def get_waiting_time(self, call_floor, target_floor):
        distance_to_reach = distance_to_block(self.position, call_floor)
        # will need to reach opposite side if wrong direction
        # distance_to_target = abs(target_floor - call_floor) # Doesn't matter as same for all lifts
        return distance_to_reach*self.time_multiplier[0] + \
               len(self.pickupQueue)*self.time_multiplier[1]*2 +\
               len(self.dropOffQueue)*self.time_multiplier[1]
        # should consider distance of queue

    def add_job(self, job, pickup=True):
        if pickup:
            self.pickupQueue.append(job)
            self.pickupQueue.sort(key=lambda x: distance_to_block(self.position, x[0]))
        else:
            self.dropOffQueue.append(job)
            self.dropOffQueue.sort(key=lambda x: distance_to_block(self.position, x))

    def reach_target(self):
        test = False
        if len(self.pickupQueue) > 0:
            test = self.position == self.pickupQueue[0][0]
        if len(self.dropOffQueue) > 0:
            test = self.position == self.dropOffQueue[0] or test
        return test

    def add_drop(self):
        if len(self.pickupQueue) > 0:
            if self.pickupQueue[0][0] == self.position:
                self.add_job(self.pickupQueue[0][1], pickup=False)
                self.pickupQueue.pop(0)
        if len(self.dropOffQueue) > 0:
            if self.dropOffQueue[0] == self.position:
                self.pickupQueue.pop(0)
