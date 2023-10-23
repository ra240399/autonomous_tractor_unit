#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class AutonomousSubscriber(Node):

    def __init__(self):
        super().__init__('autonomous_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'auto_control', 1)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'dist_transfer',
            self.autonomous_callback,
            1)
        self.subscription

        self.Kp = 1
        self.Ki = 0
        self.Kd = 0
        self.setpoint = 0
        self.prev_error = 0
        self.integral = 0
        self.max_speed = 1000

    def PID_controller(self, measured_value):
        error = measured_value - self.setpoint

        p_term = self.Kp * error

        self.integral += error 
        i_term = self.Ki * self.integral

        d_term = self.Kd * (error - self.prev_error)

        control = p_term + i_term + d_term
        self.prev_error = error
        
        return control

    def autonomous_callback(self, data):
        distance = data.data
        right_dist = distance[0]
        left_dist = distance[1]

        right_measure = self.PID_controller(right_dist)
        left_measure = self.PID_controller(left_dist)

        if right_measure == 0 or left_measure == 0:
            left_ratio = 0
            right_ratio = 0
        elif right_dist > 700 or left_dist > 700:
            left_ratio = 0
            right_ratio = 0
        else:
            left_ratio = right_measure / left_measure
            right_ratio = left_measure / right_measure

        left_speed = min(self.max_speed*left_ratio, self.max_speed)
        right_speed = min(self.max_speed*right_ratio, self.max_speed)

        msg = Float32MultiArray()

        msg.data = [float(left_speed), float(right_speed)]

        self.publisher_.publish(msg)

        # if int(data.data[3]) == 0:
            #self.get_logger().info("Publishing: %s" % str(msg.data))

def main(args=None):
    rclpy.init(args=args)

    autonomous_subscriber = AutonomousSubscriber()

    rclpy.spin(autonomous_subscriber)

    rclpy.shutdown()

if __name__ == '__main__':
    main()