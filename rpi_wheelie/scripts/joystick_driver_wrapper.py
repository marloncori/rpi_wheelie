#! /usr/bin/env python
from motor_driver import MotorDriver
import math
import rospy
#from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.smg import Joy, Range

class Wheelie(Node):
    '''Wheelie node suitalbe for a RPi robot with two PWN driven motors
    Creates listener on /command to accept string-style commands.
    Creates listener on /cmd_vel to accept twist messages.
    Creates listener on /joy to accept Xbox joystick input.

    Attributes
    ----------
    speed: float
        Speed along the X avix in meters per second; positive
        if it is forwards and negative if it is backwards
    spin: float
        Rotation about the pivot point in radians per second;
        position is clockwise when viewed from above (right spin)

    Methods
    -------
    stop()
        Stop all movement of the Wheelie robot car. '''

    def _init_(self, name, pinRightFwd, pinRightRev, pinLeftFwd, pinLeftRev,
               wheelDiameter = 0.065, wheelBase = 0.15, leftMaxRpm = 200.0,
               rightMaxRpm = 200.0, freq = 20):

        """
        Parameters
        ----------
        name: str
            Node name.

        wheel_base: float
            Distance between the center of the wheels in meters

        freq: int
            Frequency measured in Hz and used to control the PWM motors
            """
        super().__init__(name)
        self._freq = freq
        self._leftMaxRpm = leftMaxRpm
        self._rightMaxRpm = rightMaxRpm
        self._wheelDiameter = wheelDiameter
        self._wheelBase = wheelBase
        self._rightWheel = MotorDriver(pinRightFwd, pinRightRev, freq)
        self._leftWheel = MotorDriver(pinLeftFwd, pinLeftRev, freq)

        self.speed = 0.0
        self.spin = 0.0
        self.close = 0.30 #start slowing down when this close to objects
        self.stop = 0.10 #no forward motion when this close, limit_distance
        self.distance = 0.0

        self._commandSubscription = self.create_subscription(String, 'command',
                                                            self._commandCallback, 10)
        self._cmd_velSubscription = self.create_subscription(Twist, 'cmd_vel',
                                                             self._cmd_velCallback, 2)
        self._joySubscription = self.create_subscription(Joy, 'joy', self._joyCallback, 5)

        self._rangeSubscription = self.create_subscription(Range, 'range',
                                                           self._rangeCallback)

    def stop(self):
        self._leftWheel.stop()
        self._rightWheel.stop()

    def maxSpeed(self):
        rpm = (self._leftMaxRpm + self._rightMaxRpm) / 2.0
        mps = rpm * math.pi * self._wheelDiameter / 60.0
        return mps #meters per second

    def maxTwist(self):
        rps = self.maxSpeed() / self._wheelDiameter
        return rps #rotation in radians per second

    def _forward(self, speed = 100):
        self._rightWheel.forward(speed)
        self._leftWheel.forward(speed)

    def _reverse(self, speed = ):
        self._rightWheel.reverse(speed)
        self._leftWheel.reverse(speed)
        
    def _turnRight(self, speed = ):
        self._rightWheel.forward(speed)
        self._leftWheel.reverse(speed)
        
    def _turnLeft(self, speed = ):
        self._rightWheel.reverse(speed)
        self._leftWheel.forward(speed)

    def _commandCallback(self, msg):
        command = msg.data
        if command == 'forward':
            self._forward()
        elif command == 'reverse':
            self._reverse()
        elif command == 'right':
            self._turnRight()
        elif command == 'left':
            self._turnLeft()
         elif command == 'stop':
            self.stop()
        else:
            print('Unknown command, stopping instead.')
            self.stop()

    def _joyCallback(self, msg)

    '''
    Just use left joystick for now:
    LSB left/right  axes[0]  +1(left) to -1(right)
    LSB up/down     axes[1]  +1(up) to -1(down)
    LB              buttons[5] 1 pressed, 0 otherwise '''

        if abs(msg.axes[0]) > 0.10:
            self.spin = msg.axes[0]

        else:
            self.spin = 0.0

        if abs(msg.axes[1]) > 0.10:
            self.speed = msg.axes[1]

        else:
            self.speed = 0.0

        if msg.buttons[5] == 1:
            self.speed = 0
            self.spin = 0

        self._setMotorSpeeds()

    def _cmd_velCallback(self, msg):
        self.speed = msg.linear.x
        self.spin = msg.angular.z
        self._setMotorSpeeds()

    def _rangeCallback(self, msg):
        self.distance = msg.range
        self._setMotorSpeeds()

    def _setMotorSpeeds(self):
        """
        First figure out the speed ot each wheel based on spin:
        each wheel convers self._wheelBase meters in one radian,
        so the target speed for each wheel in meters per second
        is spin (radians per second) times wheelBase divided by
        wheelDiameter."""
        rightTwistMps = self.spin * self._wheelBase / self._wheelDiameter
        leftTwistMps = -1.0 * self.spin * self._wheelBase / self._wheelDiameter

        #now add this target speed in forward motion
        rightMps = self.speed + rightTwistMps
        leftMps = self.speed + leftTwistMps

        #conver mps in RPM: for each revolution, a wheel travels
        #pi * diameter meters, and each minute has 60 seconds
        leftTargetRPM = (leftMps * 60.0) / (math.pi * self._wheelDiameter)
        rightTargetRPM = (rightMps * 60.0) / (math.pi * self._wheelDiameter)

        leftPercentage = (leftTargetRPM / self._lefttMaxRpm) * 100.0
        rightPercentage = (rightTargetRPM / self._rightMaxRpm) * 100.0

        #clip to +- 100%
        leftPercentage = max(min(leftPercentage, 100.0), -100.0)
        rightPercentage = max(min(rightPercentage, 100.0), -100.0)

        #Add a governor/flag to cap forward motion when we are about
        #to bump into something (but still backwards motion being allowed)
        flag = 1.0
        if self.distance < self.stop:
            flag = 0.0
        elif self.distance < self.close:
            flag = (self.distance - self.stop) / (self.close - self.stop)

        if rightPercentage > 0:
            rightPercentage *= flag

        if leftPercentage > 0:
            leftPercentage *= flag

        self._rightWheel.run(rightPercentage)
        self._leftWheel.run(leftPercentage)

    def main(args=None):
        rospy.init(args=args)
        #rclpy.init(args=args)
        robotcar = Wheelie('wheelie', pinRightFwd = 10, pinRightRev = 9,
                           pinLeftFwd = 8, pinLeftRev = 7, lefMaxRpm = 195,
                           rightMaxRpm = 202)

        #enable keyboard controller:
        #ros2 run teleop_twist keyboard_teleop twist_keyboard
