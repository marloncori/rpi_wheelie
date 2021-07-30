#! /usr/bin/env python

from wheelie_robot import Wheelie
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,
                                                     'move', sef.listener_callback, 10)
        self.subscription   arning
        self.robot_car = Wheelie()

    def listener_callback(self, msg):
        command = msg.data
        if command == 'forward' or command == 'forwards':
            print("Moving forward")
            self.robot_car.goForward()
        elif command == 'backward' or command == 'reverse':
            print("Moving backward")
            self.robot_car.goBackward()
        elif command == 'right':
            print("Turning right")
            self.robot_car.goRight()
        elif command == 'left':
            print("Turning left")
            self.robot_car.goLeft()
        else:
            print("Unkown command. The robot car has stopped")
            self.robot_car.stop()

def main(args=None):
    rclpy.init(args=args)
    mininal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.robot_car.stop()
    gpio.cleanup()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
  
