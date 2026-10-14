import cv2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, TimeSynchronizer

from tp3_interfaces.msg import FeatureSet


class ViewFeatures(Node):

    def __init__(self):
        super().__init__('view_features')

        self.bridge = CvBridge()

        # Izquierda
        self.sub_image_left = Subscriber(
            self, Image, '/left/image_rect', 1
        )

        self.sub_features_left = Subscriber(
            self, FeatureSet, '/left/features'
        )

        self.sync_left = TimeSynchronizer(
            [ self.sub_image_left,
            self.sub_features_left],
            queue_size=30,
        )

        self.sync_left.registerCallback(
            self.procesar_left
        )

        # Derecha
        self.sub_image_right = Subscriber(
            self, Image, '/right/image_rect'
        )

        self.sub_features_right = Subscriber(
            self, FeatureSet, '/right/features'
        )

        self.sync_right = TimeSynchronizer(
            [ self.sub_image_right,
            self.sub_features_right],
            queue_size=30,
        )

        self.sync_right.registerCallback(self.procesar_right)


    def dibujar_features(self, image_msg, features_msg):

        imagen = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8'
        )

        for punto in features_msg.keypoints:

            x = int(round(punto.x))
            y = int(round(punto.y))

            cv2.circle(imagen, (x, y), 3, (0, 255, 0), -1)

        return imagen


    def procesar_left(self, image_msg, features_msg):

        imagen = self.dibujar_features(image_msg, features_msg)

        cv2.imshow('Features izquierda', imagen)

        cv2.waitKey(1)


    def procesar_right(self, image_msg, features_msg):

        imagen = self.dibujar_features(image_msg, features_msg)

        cv2.imshow('Features derecha', imagen)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = ViewFeatures()

    try:

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

            cv2.waitKey(1)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == '__main__':
    main()