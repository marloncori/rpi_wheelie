#! /usr/bin/env python

from wheelie_robot import Wheelie
import rospy
from std_msgs.msg import String
import time

robot_car = Wheelie()

def listener_callback(self, msg):
    command = msg.data
    if command == 'forward' or command == 'forwards':
        print("Moving forward")
        robot_car.goForward()
    elif command == 'backward' or command == 'reverse':
        print("Moving backward")
        robot_car.goBackward()
    elif command == 'right':
        print("Turning right")
        robot_car.goRight()
    elif command == 'left':
        print("Turning left")
        robot_car.goLeft()
    else:
        print("Unkown command. The robot car has stopped")
        robot_car.stop()
        
def minimal_subscriber:
    rospy.init_node('robot_whelie', anonymous=True)
    rospy.Subscribe('move', String, listener_callback, queue_size=10)
    rospy.spin()

def shutdown():
    robot_car.end()
    
if __name__ == '__main__':
    try:
        minimal_subscriber()
    except rospy.ROSInterrupt:
        rospy.loginfo("User has finished program execution.")
        shutdown()
