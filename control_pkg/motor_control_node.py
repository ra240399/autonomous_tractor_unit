#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import serial
import subprocess
import time
import re
import openpyxl

class MotorControlSubscriber(Node):

    def __init__(self):
        super().__init__('motor_control_subscriber')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'battery_data', 10)

        self.main_subscription = self.create_subscription(
            Float32MultiArray,
            'dist_transfer',
            self.main_callback,
            1)
        self.main_subscription

        self.autonomous_subscription = self.create_subscription(
            Float32MultiArray,
            'auto_control',
            self.autonomous_callback,
            1)
        self.autonomous_subscription

        self.manual_subscription = self.create_subscription(
            Float32MultiArray,
            'manual_control',
            self.manual_callback,
            1)
        self.manual_subscription

        self.obstacle_subscription = self.self.create_subscription(
            Float32,
            'obstacle_detection',
            self.obstacle_callback,
            1)

        self.auto_speed = []
        self.manual_speed = []
        self.obstacle_distance = 0
        self.ser = serial.Serial(port='/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_AUCWb11BS14-if00-port0', baudrate=115200, bytesize=8, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
        self.ser2 = serial.Serial(port='/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_CYABb11BS14-if00-port0', baudrate=115200, bytesize=8, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
        self.flag = 1
        self.max_ratio = 1023
        self.read_list = []
        self.read2_list = []


        self.start_time = time.time()


    def reset_usb(self):
        try:
            command = 'uhubctl -l 1-1.3 -a off'
            result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            time.sleep(2)
            command = 'uhubctl -l 1-1.3 -a on'
            result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if result.returncode == 0:
                print("usb reset correctly")
            else:
                print("usb reset failed")

            set_time = time.time()
            for i in range(100000000):
                i += 1

            time_taken = time.time() - set_time
            print(time_taken)
        
        except Exception as e:
            print(f"An error occured: {e}")

    def write_command(self, motor1_command, motor2_command):
        self.ser.write(motor1_command.encode('ascii'))
        self.ser2.write(motor2_command.encode('ascii'))

    def autonomous_callback(self, msg):
        self.auto_speed = msg.data

    def manual_callback(self, msg):
        self.manual_speed = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_dist = msg.data

    def main_callback(self, msg):
        mode = int(msg.data[3])
        picking_ratio = (msg.data[2]/self.max_ratio)
        # print(picking_ratio)

        if mode == 0:
            if len(self.auto_speed) == 2:
                if self.obstacle_distance >= 5000.0 or self.obstacle_distance == 0:
                    left_speed = self.auto_speed[0] * picking_ratio
                    right_speed = self.auto_speed[1] * picking_ratio
                    motor_speed = [left_speed, right_speed]
                elif self.obstacle_distance < 5000.0 and self.obstacle_distance > 2000.0:
                    picking_ratio = picking_ratio/2
                    left_speed = self.auto_speed[0] * picking_ratio
                    right_speed = self.auto_speed[1] * picking_ratio
                    motor_speed = [left_speed, right_speed]
                elif self.obstacle_distance <= 2000.0:
                    motor_speed = [0, 0]
            else:
                motor_speed = [0, 0]
        else:
            motor_speed = self.manual_speed

        if len(motor_speed) == 2:
            emergency = int(msg.data[4])

            if emergency:
                self.flag = 1

            if not emergency and self.flag:
                self.flag = 0
                self.ser.close()
                self.ser2.close()
                self.reset_usb()
                self.ser.open()
                self.ser2.open()
                

            #print(emergency)
            if not emergency and not self.flag:
                #print(motor_speed)
                self.ser.flushInput()
                self.ser2.flushInput()

                # max speed is 1000 which is 1500rpm 
                # this piece controls the speed
                command = '!M {}\r'.format(-1*motor_speed[0])
                command2 = '!M {}\r'.format(motor_speed[1])
                self.write_command(command, command2)

                volts_command = '?V\r'
                self.write_command(volts_command, volts_command)

                motor_amps_command = '?A\r'
                self.write_command(motor_amps_command, motor_amps_command)

                batt_amps_command = '?BA\r'
                self.write_command(batt_amps_command, batt_amps_command)

                count1 = 0
                while count1 < 8:
                    read = self.ser.read().decode()
                    self.read_list.append(read)
                    if read == '\r':
                        count1 += 1
                        print(count1)

                count2 = 0
                while count2 < 8:
                    read2 = self.ser2.read().decode()
                    self.read2_list.append(read2)
                    if read2 == '\r':
                        count2 += 1
                        print(count2)

                self.read_list = [item for item in self.read_list if item != '\r'] 
                self.read2_list = [item for item in self.read2_list if item != '\r'] 

                motor1_string = ''.join(self.read_list)
                motor2_string = ''.join(self.read2_list)

                volt1_match = re.search(r'V=([\d:]+)', motor1_string)
                volt2_match = re.search(r'V=([\d:]+)', motor2_string)

                motor1_amp_match = re.search(r'AA=(-?\d+)', motor1_string)
                motor1_batt_match = re.search(r'BA=(-?\d+)', motor1_string)

                motor2_amp_match = re.search(r'AA=(-?\d+)', motor2_string)
                motor2_batt_match = re.search(r'BA=(-?\d+)', motor2_string)

                voltage1_data = []
                if volt1_match:
                    voltage1_data = volt1_match.group(1).split(':')
                    int_volt = float(float(voltage1_data[0])/10)
                    batt_volt = float(float(voltage1_data[2])/100)
                    motor1_volt = float(float(voltage1_data[1])/10)

                voltage2_data = []
                if volt2_match:
                    voltage2_data = volt2_match.group(1).split(':')
                    motor2_volt = float(float(voltage2_data[1])/10)


                if motor1_amp_match: 
                    motor1_amps = float(abs((float(motor1_amp_match.group(1)))/10))
                if motor1_batt_match:
                    motor1_batt = float(abs((float(motor1_batt_match.group(1))/10)))

                if motor2_amp_match: 
                    motor2_amps = float(abs((float(motor2_amp_match.group(1)))/10))
                if motor2_batt_match:
                    motor2_batt = float(abs((float(motor2_batt_match.group(1)))/10))

                current_time = float(time.time() - self.start_time)

                L_speed = float(motor_speed[0])
                R_speed = float(motor_speed[1])

                data = [current_time, int_volt, batt_volt, motor1_volt, motor1_amps, motor1_batt, motor2_volt, motor2_amps, motor2_batt, L_speed, R_speed]

                print(data)

                self.read_list = []
                self.read2_list = []

                msg_batt = Float32MultiArray()

                msg_batt.data = data

                self.publisher_.publish(msg_batt)


def main(args=None):
    rclpy.init(args=args)

    motor_control_subscriber = MotorControlSubscriber()

    rclpy.spin(motor_control_subscriber)

    # motor_control_subscriber.save_file()
    # print("saved")

    rclpy.shutdown()


if __name__ == '__main__':
    # rclpy.init(args=None)

    # motor_control_subscriber = MotorControlSubscriber()

    # rclpy.spin(motor_control_subscriber)

    # motor_control_subscriber.save_file()
    # print("saved")

    # rclpy.shutdown()
    main()