from utility import *


class LiftCar:
    def __init__(self, car_id, position):
        # self.jobQueue = []
        self.ID = car_id
        self.position = position
        self.status = 0
        # 0 Stay/1 Proceed/2 Waiting/3 Servo operating/8 Loading/9 Reached
        self.blockQueue = []
        self.pickupQueue = []
        self.dropOffQueue = []
        self.time_multiplier = [1, 1]  # multiplier for distance calculation for travel time and stop time

    def get_waiting_time(self, call_floor, target_floor):
        distance_to_reach = distance_to_floor(self.position, call_floor, target_floor)
        return distance_to_reach * self.time_multiplier[0] + \
               len(self.pickupQueue) * self.time_multiplier[1] * 2 + \
               len(self.dropOffQueue) * self.time_multiplier[1]
        # should consider distance of queue

    def add_job(self, job, pickup=True):
        if pickup:
            self.pickupQueue.append([job[0], job[1]] if job[1] > job[0] else [17-job[0], 17-job[1]])
            self.pickupQueue.sort(key=lambda x: (x[0] + 18 - self.position) % 18)
        else:
            self.dropOffQueue.append(job)
            self.dropOffQueue.sort(key=lambda x: (x + 18 - self.position) % 18)

    def job_count(self):
        return len(self.pickupQueue)+len(self.dropOffQueue)

    def reach_target(self):
        test = False
        if len(self.pickupQueue) > 0:
            test = self.position == self.pickupQueue[0][0]
        if len(self.dropOffQueue) > 0:
            test = self.position == self.dropOffQueue[0] or test
        return test

    def loading(self):
        for u in list(dict.fromkeys([job[1] for job in self.pickupQueue if job[0] == self.position])):
            self.add_job(self.pickupQueue[0][1], pickup=False)
        self.pickupQueue = [job for job in self.pickupQueue if job[0] != self.position]
        self.dropOffQueue = [job for job in self.dropOffQueue if job != self.position]