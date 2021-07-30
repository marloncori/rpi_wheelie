#! /usr/bin/env python
import RPi.GPIO as rpi
import time

rpi.setmode(rpi.BCM)
rpi.setwarnings(False)

class MotorDriver:

    def __init__(self, pinFwd, pinBkw, frequency=20, maxSpeed=100):
        rpi.setup(pinFwd, rpi.OUT)
        rpi.setup(pinBack, rpi.OUT)

        self._frequency = frequency
        self._maxSpeed = maxSpeed
        self._pwmFwd = rpi.PWM(pinFwd, frequency)
        self._pwmBkw = rpi.PWM(pinBkw, frequency)
        self.stop()

    def forward(self, speed):
        self._move(speed)

    def reverse(self, speed):
        self._move(-speed)

    def stop(self):
        self._move(0)

    def _move(self, speed):
        #limits
        if speed > self._maxSpeed:
            speed = self._maxSpeed
        if speed < -self._maxSpeed:
            speed = -self._maxSpeed

        if speed < 0:
            self._pwmFwd.start(0) #gpio pwm function
            self._pwmBkw.start(-speed)
        else:
            self._pwmFwd.start(speed)
            self._pwmBkw.start(0)

        
