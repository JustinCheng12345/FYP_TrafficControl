#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Traffic Control for Cheng Yu Tin, Justin's FYP, Ropeless Lift System.

This programme should acquire data from both MQTT and seriall,
and then to calculate the optimal path for the lift cars. Signals will
then be sent to different controllers using MQTT or serial communication.
"""

import serial
import time
from car import LiftCar
import paho.mqtt.client as mqtt

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

        self.blocks = [False]*18
        self.sensors = [False] * 36
        self.cars = []
        self.userQueue = []
        self.signalQueue = []
        self.routineControl = False
        self.servo = [True, True]

    def extract(self):
        # to extract sensor data from the Arduino mega  
        self.mega.write(b"se00get\n")
        tempblocks = [False]*18
        for trueblock in self.mega.readline().decode("utf-8").split(","):
            tempblocks[int(trueblock)] = True
        self.blocks = [new if not new else old for old, new in zip(self.blocks, tempblocks)]
        print(self.blocks)
        for car in self.cars:
            car.position = self.blocks.index(car.ID)
        #for index, block in enumerate(newblocks):
        #   self.blocks[index] = self.blocks[index] if block is not False else False

    def calculate(self):
        # to calculate the optimal path for each lift
        # apply user queue to job queue
        for u in self.userQueue:
            print(u)
            time_queue = [t.get_waiting_time(u[0], u[1]) for t in self.cars]
            self.cars[time_queue.index(min(time_queue))].add_job(u)
            self.userQueue.remove(u)

        for car in self.cars:
            print(car.pickupQueue)
            # If it is possible to move to next floor
            next_block = self.blocks[(car.position+1) % 18]
            if car.reach_target():
                car.status = 9
            elif next_block is False:
                self.blocks[(car.position+1) % 18] = car.ID
            else:
                car.status = 0

            # Loading
            if car.status == 9:
                car.addDrop()
                if car.pickupQueue[0][0] == car.position:
                    car.add_job(car.pickupQueue[0][1], pickup= False)
                    car.pickupQueue.pop(0)
                if car.dropoffQueue[0] == car.position:
                    car.pickupQueue.pop(0)
        # calculate route
        pass

    def signal(self):
        # to provide signal for each car
        for car in self.cars:
            if self.blocks.count(car.ID) > 1:
                if (car.position == 0 or car.position == 16) and self.servo[0] is False:
                    # self.mqttClient.publish("Signal", "sclonormal")
                    self.mega.write(b"sclonormal\n")
                elif (car.position == 7 or car.position == 9) and self.servo[0] is False:
                    # self.mqttClient.publish("Signal", "scupnormal")
                    self.mega.write(b"scupnormal\n")
                elif (car.position == 17) and self.servo[0] is True:
                    # self.mqttClient.publish("Signal", "sclotransfer")
                    self.mega.write(b"sclotransfer\n")
                elif (car.position == 8) and self.servo[0] is True:
                    # self.mqttClient.publish("Signal", "sclotransfer")
                    self.mega.write(b"sclotransfer\n")
                elif car.position < 9:
                    self.mqttClient.publish("Signal", "lc"+car.ID+"go")
                else:
                    self.mqttClient.publish("Signal", "lc"+car.ID+"down")
            else:
                self.mqttClient.publish("Signal", "lc"+car.ID+"stop")
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
        print(message)
        if (message.topic == "User"):
            self.userQueue.append([int(n) for n in message.payload.decode("utf-8").split(",")])


def main():
    print("start")
    # handle serial connection with mega, will reset mega
    tc = TrafficController()
    for i in range(0, int(input("Number of cars?"))):
        x = input("ID/position of car "+str(i)+" :")
        tc.add_car(x[:2], int(x[3:]))
    tc.routine()


if __name__ == '__main__':
    main()
