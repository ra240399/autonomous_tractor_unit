# @software{yolov8_ultralytics,
#   author = {Glenn Jocher and Ayush Chaurasia and Jing Qiu},
#   title = {Ultralytics YOLOv8},
#   version = {8.0.0},
#   year = {2023},
#   url = {https://github.com/ultralytics/ultralytics},
#   orcid = {0000-0001-5950-6979, 0000-0002-7603-6750, 0000-0003-3783-7069},
#   license = {AGPL-3.0}
# }


import rclpy
from rclpy.node import (Node)

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ultralytics import YOLO

from std_msgs.msg import Float32

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subcriber')
        self.model = YOLO('/home/jetson/autonomous_tractor_ws/src/yolobot_recognition/scripts/yolov8n.pt')
        self.rgb_subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.rgb_listener_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/depth/image_rect_raw',
            self.depth_listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'obstacle_detection', 1)
        self.rgb_subscription  # prevent unused variable warning
        self.depth_subscription
        self.centre_coordinates = []

    def rgb_listener_callback(self, msg):
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            dim = (848,480)
            cv_image = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)


            results = self.model(cv_image)
            #print(results)

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    if self.model.names[int(box.cls)] == 'person':
                        b = box.xyxy[0].to('cpu').detach().numpy().copy()
                        top_left = (int(b[0]), int(b[1]))
                        bottom_right = (int(b[2]), int(b[3]))
                        centre_x = (top_left[0] + bottom_right[0]) // 2
                        centre_y = (top_left[1] + bottom_right[1]) // 2
                        self.centre_coordinates = (centre_x, centre_y)
                        cv2.circle(cv_image, self.centre_coordinates, 10, (0,0,255), -1)
                        cv2.rectangle(cv_image, top_left, bottom_right,(255,255,0))


            cv2.imshow("Raw Camera Image", cv_image)
            key = cv2.waitKey(1)

            if key == ord('q'):
                # cv2.imwrite('field photo.png', cv_image)
                pass

            cv2.imwrite('field photo2.png', cv_image)

        except CvBridgeError:
            #rclpy.logerr("CvBridge Error (Raw Image): %s", e)
            print("error")

    def depth_listener_callback(self, msg):
        try:
            bridge = CvBridge()
            cv_depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


            if len(self.centre_coordinates) == 2:
                depth = cv_depth_image[self.centre_coordinates[1]][self.centre_coordinates[0]]
                print(depth)
            # cv2.imshow("Raw depth Image", cv_depth_image)
            # key = cv2.waitKey(1)

            # if key == ord('q'):
            #     pass

            depth_data = Float32()
            depth_data.data = float(depth)
            self.publisher_.publish(depth_data)

        except CvBridgeError:
            #rclpy.logerr("CvBridge Error (Raw Image): %s", e)
            print("error")


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()
