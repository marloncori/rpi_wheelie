from motor_driver import MotorDriver
from time import sleep

class Wheelie:
    def __init__(self):
        self.rightWheel = MotorDriver('d:10:p','d:12:p', 'd:9:o')
        self.leftWheel = MotorDriver('d:8:p', 'd:7:p', 'd:6:o')

    def stop(self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def goForward(self, speed=80)
        self.rightWheel.forward(speed)
        self.leftWheel.forward(speed)

    def goBackward(self, speed=80)
        self.rightWheel.reverse(speed)
        self.leftWheel.reverse(speed)

    def goLeft(self, speed=80)
        self.rightWheel.reverse(speed)
        self.leftWheel.forward(speed)
        
    def goRight(self, speed=80)
        self.rightWheel.forward(speed)
        self.leftWheel.reverse(speed)
        
    def shutdown(self):
        self.rightWheel.finish()
        self.leftWheel.finish()

robotcar = Wheelie()
def run():

    robotcar.goForward(70)
    sleep(1)
    robotcar.goRight(60)
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
        robotcar.shutdown()
