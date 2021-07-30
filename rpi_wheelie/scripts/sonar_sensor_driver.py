#!/usr/bin/env python
from time import sleep
from ultrassonic_sensor import UltrassonicSensor

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class DistanceSensor(Node):

    def __init__(self, name, trigPin, echoPin, freq = 10)
        super().__init__(name)

        self._frequency = freq
        self._distance_sensor = UltrassonicSensor(trigPin, echoPin)
        self._range_msg = Range()
        self._range_msg.radiation_type = Range.ULTRASOUND
        self._range_msg.field_of_view = self._distance_sensor.angle
        self._range_msg.min_range = self._distance_sensor.minRange
        self._range_msg.max_range = self._distance_sensor.maxRange

        self.start()

    def _distanceCallback(self):
        self.distance = self._distance_sensor.getDistance()
        self._range_msg.range = self.distance
        self._distance_publisher.publish(self._range_msg)

    def stop(self):
        self.destroy_timer(self._distance_timer)

    def start(self):
        self.distance_timer = self.create_timer(1.0 / self._frequency,
                                                self._distanceCallback)

def main(args=None):
    rclpy.init(args=args)
    hc_sr04 = DistanceSensor("range", trigPin = 17, echoPin = 18)
    print("Spinning")
    rclpy.spin(hc_sr04)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        print("The program has been finished")


















        
