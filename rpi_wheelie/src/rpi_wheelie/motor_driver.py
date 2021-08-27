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
        return round(o_min + (o_max - o_min) * ((val - i_min) / (i_max - i_min)))
    
    def _speed_map_abs(self, val, i_min, i_max, o_min, o_max):
        return abs(round(o_min + (o_max - o_min) * ((val - i_min) / (i_max - i_min))))
    
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
        if speed > 1.0:
            speed = 1.0
        if speed < -1.0:
            speed = -1.0

        if speed < 0.0:
            self._pwm = self._speed_map_abs(speed, -1.0, 0.0, self._minSpeed, self._maxSpeed)
            for spin in range(self._minSpeed, self._pwm, self._frequency):
                self.pinF.write(0)
                self.pinB.write(spin)
                self.pinE.write(1)
                sleep(0.04)
        else:
            self._pwm = self._speed_map(speed, 0.0, 1.0, self._minSpeed, self._maxSpeed)
            for spin in range(self._minSpeed, self._pwm, self._frequency):
                self.pinF.write(spin)
                self.pinB.write(0)
                self.pinE.write(1)
                sleep(0.04)
