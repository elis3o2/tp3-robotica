import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


class Disparity(Node):

    def __init__(self):
        super().__init__('disparity')

        self.bridge = CvBridge()

        # Matcher estéreo
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=15
        )

        # Imagen izquierda
        self.sub_left = Subscriber(
            self, Image, '/left/image_rect'
        )

        # Imagen derecha
        self.sub_right = Subscriber(
            self, Image, '/right/image_rect'
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_left,
            self.sub_right],
            queue_size=30,
            slop=0.05
        )

        self.sync.registerCallback(
            self.procesar
        )

        # Mapa de disparidad
        self.pub = self.create_publisher(
            Image, '/disparity', 10
        )

        self.get_logger().info(
            'Nodo disparity iniciado'
        )


    def procesar(self, msg_left: Image, msg_right: Image):

        imagen_left = self.bridge.imgmsg_to_cv2(
            msg_left,
            desired_encoding='mono8'
        )

        imagen_right = self.bridge.imgmsg_to_cv2(
            msg_right,
            desired_encoding='mono8'
        )

        # Calcular disparidad
        disparity = self.stereo.compute(
            imagen_left,
            imagen_right
        )

        # StereoSGBM devuelve disparidad multiplicada por 16
        disparity = disparity.astype(
            np.float32
        ) / 16.0

        # Convertir a mensaje ROS
        msg_disparity = self.bridge.cv2_to_imgmsg(
            disparity,
            encoding='32FC1'
        )

        # Conservar timestamp y frame
        msg_disparity.header = msg_left.header

        self.pub.publish(
            msg_disparity
        )


def main(args=None):

    rclpy.init(args=args)

    node = Disparity()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()