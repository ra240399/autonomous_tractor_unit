#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import openpyxl
import time

class DataSubscriber(Node):

    def __init__(self):
        super().__init__('data_subscriber')

        self.folder_path = "/home/jetson/autonomous_tractor_data/sensor_data/"
        self.file_name = "sensor_data_straight.xlsx"
        self.file = self.folder_path + self.file_name

        self.workbook = openpyxl.Workbook()
        self.workbook.save(self.file)
        self.worksheet = self.workbook.active

        self.start_time = time.time()

        self.main_subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.data_callback,
            1)
        self.main_subscription
        

    def data_callback(self, data):

        try:
            # data_list = [data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5],data.data[6],data.data[7],data.data[8],
            # data.data[9],data.data[10]]
            current_time = time.time() - self.start_time

            data_list = [current_time, data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5]]
            self.worksheet.append(data_list)
            self.workbook.save(self.file)

        # workbook.close()
        # print(data.data)

        except:
            self.workbook.save(self.file)
            self.workbook.close()

def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    rclpy.spin(data_subscriber)

    rclpy.shutdown()

if __name__ == '__main__':

    main()
