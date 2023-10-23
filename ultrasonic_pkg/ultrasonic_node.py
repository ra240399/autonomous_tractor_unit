#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class UltrasonicPublisher(Node):

    def __init__(self, ser):
        super().__init__('ultrasonic_publisher')
        self.sensor_publisher = self.create_publisher(Float32MultiArray, 'sensor_data', 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'dist_transfer', 1)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.dist_callback)
        self.ser = ser

    def calc_dist(self):
        try:
            self.ser.flushInput()

            # read the data from arduino over the serial
            line = self.ser.readline().decode().strip()
            data = line.split(",")
            
            left_dist = float(data[0])
            right_dist = float(data[1])
            speed = float(data[2])
            switch = float(data[3])
            emergency = float(data[4])
            left_front = float(data[5])
            left_back = float(data[6])
            right_front = float(data[7])
            right_back = float(data[8])

            return [left_dist, right_dist, speed, switch, emergency, left_front, left_back, right_front, right_back]
        
        except serial.SerialException:

            return [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def dist_callback(self):
        sensor_msg = Float32MultiArray()
        msg = Float32MultiArray()
        dist_data = self.calc_dist()
        dist_transfer = [dist_data[0], dist_data[1], dist_data[2], dist_data[3], dist_data[4]]
        sensor_transfer = [dist_data[0], dist_data[1], dist_data[5], dist_data[6], dist_data[7], dist_data[8]]

        msg.data = dist_transfer
        sensor_msg.data = sensor_transfer

        self.publisher_.publish(msg)
        self.sensor_publisher.publish(sensor_msg)
        # print(msg.data)
        print(sensor_msg.data)
        #self.get_logger().info("Publishing: %s" % str(msg.data))
        # time.sleep(0.05)

def main(args=None):
    ser = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_7513030383535170D0B2-if00', 115200)

    rclpy.init(args=args)

    ultrasonic_publisher = UltrasonicPublisher(ser)

    # ultrasonic_publisher.dist_callback()

    rclpy.spin(ultrasonic_publisher)

    ultrasonic_publisher.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()