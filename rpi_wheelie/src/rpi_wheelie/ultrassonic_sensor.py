#! /usr/bin/env python

import RPi.GPIO as rpi
from time import sleep
import math

class UltrassonicSensor:

    def __init__(self, trigPin, echoPin):
        rpi.setmode(rpi.BCM)
        rpi.setwarnings(False)
        rpi.setup(trigPin, rpi.OUT)
        rpi.setup(echoPin, rpi.IN)

        self._trigPin = trigPin
        self._echoPin = echoPin
        self._speedOfSound = 343.0

        self.minRange = 0.030
        self.maxRange = 4.000
        self.angle = 15.0 * math.pi / 180.0

        #pre-compute some static values
        self.minTravelTime = 2.0 * self.minRange / self.speedOfSound
        self.maxTravelTime = 2.0 * self.maxRange / self.speedOfSound

    def getDistance():
        rpi.output(trigPin, True)
        sleep(0.00001)
        rpi.output(trigPin, False)

        startTime = time.time()
        while rpi.input(echoPin) == 0:
            startTime = time.time()

        stopTime = time.time()
        while rpi.input(echoPin) == 1:
            stopTime = time.time()
            if stopTime - startTime >= 0.04:
                stopTime = startTime
                break

        elapsedTime = stopTime - startTime
        distance_cm = self._milisecsToCentimeter(elapsedTime)
        #distance_in = distance_cm * 0.393701

        return distance_cm

    def _milisecsToCentimeter(duration):
        distance = (duration/29)/2
        
        return distance

    def _milisecsToInches(time):
        proximity = (duration * 0.393701

        return proximity
        
