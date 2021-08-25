#! /usr/bin/env python3
from pyfirmata import Arduino, util
from time import sleep

class MotorDriver(Arduino):

    def __init__(self, fwd_pin, bck_pin, frequency=20, minSpeed=0, maxSpeed=100):
        
        self._positive = fwd_pin
        self._negative = bck_pin
        self._enable = en_pin

        self._frequency = frequency
        self._minSpeed = minSpeed
        self._maxSpeed = maxSpeed
        self._pwm = None
        
        self.uno = Arduino('/dev/ttyACM0')
        self.it = util.Iterator(self.uno)
        self.pinF = self.uno.get_pin(self._positive)
        self.pinB = self.uno.get_pin(self._negative)
        self.pinE = self.uno.get_pin(self._enable)
 
    def _speed_map(self, val, i_min, i_max, o_min, o_max):
        self._pwm = round(o_min + (o_max - o_min) * ((val - i_min) / (i_max - i_min)))
        return self._pwm
    
    def _speed_map_abs(self, val, i_min, i_max, o_min, o_max):
        self._pwm = abs(round(o_min + (o_max - o_min) * ((val - i_min) / (i_max - i_min))))
        return self._pwm
    
    def forward(self, speed):
        self._move(speed)

    def reverse(self, speed):
        self._move(-speed)

    def stop(self):
        self._move(0)

    def finish(self):
        self._move(0)
        self.uno.exit()
        
    def _move(self, speed):
        #limits
        if speed > self._maxSpeed:
            speed = self._maxSpeed
        if speed < -self._maxSpeed:
            speed = -self._maxSpeed

        if speed < 0:
            self._speed_map_abs(speed, -self._maxSpeed, self._minSpeed, 0, 256)
            for spin in range(self._minSpeed, self._pwm, self._frequency):
                self.pinF.write(0)
                self.pinB.write(self._pwm)
                self.pinE.write(1)
                sleep(0.03)
        else:
            self._speed_map(speed, self._minSpeed, self._maxSpeed, 0, 256)
            for spin in range(self._minSpeed, self._pwm, self._frequency):
                self.pinF.write(self._pwm)
                self.pinB.write(0)
                self.pinE.write(1)
                sleep(0.03)
