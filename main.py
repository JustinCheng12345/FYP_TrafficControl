#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Traffic Control for Cheng Yu Tin, Justin's FYP, Ropeless Lift System.

This programme should acquire data from both MQTT and seriall,
and then to calculate the optimal path for the lift cars. Signals will
then be sent to different controllers using MQTT or serial communication.
"""

import time

import paho.mqtt.client as mqtt
import serial

from car import LiftCar


class TrafficController:
    def __init__(self):
        self.mega = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(1)
        self.mega.flush()

        self.mqttClient = mqtt.Client(client_id="ctc")
        self.mqttClient.connect("127.0.0.1")
        self.mqttClient.subscribe("Status")
        self.mqttClient.subscribe("User")
        self.mqttClient.on_message = self.on_message

        self.blocks = [False] * 18
        self.sensors = [False] * 36
        self.cars = []
        self.userQueue = []
        self.signalQueue = []
        self.routineControl = False
        self.servo = [True, True]  # true when normal

    def extract(self):
        # to extract sensor data from the Arduino mega  
        self.mega.write(b"se00get\n")
        tempblocks = [False] * 18
        for trueblock in self.mega.readline().decode("utf-8").split(",")[:-1]:
            tempblocks[int(trueblock)] = True
        self.blocks = [new if not new else old for old, new in zip(self.blocks, tempblocks)]
        for car in self.cars:
            car.position = self.blocks.index(car.ID)

    def calculate(self):
        # to calculate the optimal path for each lift
        # apply user queue to job queue
        for u in self.userQueue:
            time_queue = [t.get_waiting_time(u[0], u[1]) for t in self.cars]
            self.cars[time_queue.index(min(time_queue))].add_job(u)
            self.userQueue.remove(u)

        for car in self.cars:
            # If it is possible to move to next floor
            if car.reach_target():
                car.status = 9
                car.loading()
            elif car.status == 8 or car.status == 3:
                pass
            else:
                if car.job_count() > 0:
                    if self.blocks[(car.position + 1) % 18] is False and\
                            (self.blocks[(car.position + 2) % 18] is False or
                             not (car.position == 16 or car.position == 7)):
                        self.blocks[(car.position + 1) % 18] = car.ID
                        car.status = 1
                    else:
                        car.status = 2
                        if self.blocks[car.position+1]:
                            previous_car = [x for x in self.cars if x.ID == self.blocks[car.position+1]][0]
                            if previous_car.status == 0:
                                previous_car.add_job((previous_car.location + 3) % 18, pickup=False)
                else:
                    car.status = 0
        # calculate route
        pass

    def signal(self):
        # to provide signal for each car
        for car in self.cars:
            if car.status == 9:
                self.mqttClient.publish("Signal", "lc" + car.ID + "load")
                car.status = 8
            if self.blocks.count(car.ID) > 1:
                if (car.position == 0 or car.position == 16) and self.servo[0] is False:
                    # self.mqttClient.publish("Signal", "sclonormal")
                    self.mega.write(b"sclonormal\n")
                    self.mqttClient.publish("Signal", "lc" + car.ID + "servo")
                    self.servo[0] = True
                elif (car.position == 7 or car.position == 9) and self.servo[1] is False:
                    # self.mqttClient.publish("Signal", "scupnormal")
                    self.mega.write(b"scupnormal\n")
                    self.mqttClient.publish("Signal", "lc" + car.ID + "servo")
                    self.servo[1] = True
                elif (car.position == 17) and self.servo[0] is True:
                    # self.mqttClient.publish("Signal", "sclotransfer")
                    self.mega.write(b"sclotransfer\n")
                    self.mqttClient.publish("Signal", "lc" + car.ID + "servo")
                    self.servo[0] = False
                elif (car.position == 8) and self.servo[9] is True:
                    # self.mqttClient.publish("Signal", "scuptransfer")
                    self.mega.write(b"scuptransfer\n")
                    self.mqttClient.publish("Signal", "lc" + car.ID + "servo")
                    self.servo[1] = False
                elif car.position < 9:
                    self.mqttClient.publish("Signal", "lc" + car.ID + "up")
                else:
                    self.mqttClient.publish("Signal", "lc" + car.ID + "down")
            else:
                self.mqttClient.publish("Signal", "lc" + car.ID + "stop")
        pass

    def maintenance(self):
        self.mqttClient.loop(timeout=0.1)

    def routine(self):
        while True:
            self.extract()
            self.calculate()
            self.signal()
            self.maintenance()
            time.sleep(0.1)

    def add_car(self, car_id, position):
        self.cars.append(LiftCar(car_id, position))
        self.blocks[position] = car_id

    def on_message(self, client, userdata, message):
        # print("%s %s" % (message.topic, message.payload))
        if message.topic == "User":
            self.userQueue.append([int(n)-1 for n in message.payload.decode("utf-8").split(",")])
            # -1 as first floor = floor 0
        if message.topic == "Status":
            m = message.payload.decode("utf-8")
            if m[2:] == "finloading":
                [car for car in self.cars if car.ID == m[0:2]][0].status = 0
            if m[2:] == "finservo":
                [car for car in self.cars if car.ID == m[0:2]][0].status = 0


def main():
    print("start")
    # handle serial connection with mega, will reset mega
    tc = TrafficController()
    for i in range(0, int(input("Number of cars?"))):
        x = input("ID/position of car " + str(i) + " :")
        tc.add_car(x[:2], int(x[3:]))
    tc.routine()


if __name__ == '__main__':
    main()
