from motor_driver import MotorDriver
import RPi.GPIO as gpio
from time import sleep

class Wheelie:
    def __init__(self):
        self.rightWheel = MotorDriver(10, 9)
        self.leftWheel = MotorDriver(8, 7)

    def stop(self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def goForward(self, speed=100)
        self.rightWheel.forward(speed)
        self.leftWheel.forward(speed)

    def goBackward(self, speed=100)
        self.rightWheel.reverse(speed)
        self.leftWheel.reverse(speed)

    def goLeft(self, speed=100)
        self.rightWheel.reverse(speed)
        self.leftWheel.forward(speed)
        
    def goRight(self, speed=100)
        self.rightWheel.forward(speed)
        self.leftWheel.reverse(speed)

def run():
    robotcar = Wheelie()
    robotcar.goForward(70)
    sleep(1)
    robotcar.goRight()
    sleep(1)
    robotcar.goBackward(50)
    sleep(1)
    robotcar.goLeft(80)
    sleep(1)
    robotcar.stop()
    sleep(1)

if __name__ == '__main__':
    try:
        while True:
            run()
    except KeyboardInterrupt:
        print("Master has stopped the robot car")
        gpio.cleanup()
