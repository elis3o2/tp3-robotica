import cv2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tp3_interfaces.msg import MatchingSet


class ViewMatchings(Node):

    def __init__(self):
        super().__init__('view_matchings')

        self.bridge = CvBridge()

        self.sub_image_left = Subscriber(
            self, Image, '/left/image_rect'
        )

        self.sub_image_right = Subscriber(
            self, Image, '/right/image_rect'
        )

        self.sub_matchings = Subscriber(
            self, MatchingSet, '/matchings'
        )

        self.sync = ApproximateTimeSynchronizer(
            [ self.sub_image_left,
            self.sub_image_right,
            self.sub_matchings],
            queue_size=30,
            slop=0.05
        )

        self.sync.registerCallback(
            self.procesar
        )


    def procesar(self, image_left_msg, image_right_msg, matchings_msg):

        imagen_left = self.bridge.imgmsg_to_cv2(
            image_left_msg,
            desired_encoding='bgr8'
        )

        imagen_right = self.bridge.imgmsg_to_cv2(
            image_right_msg,
            desired_encoding='bgr8'
        )

        cantidad = 0

        for punto_left, punto_right, distance in zip(
            matchings_msg.keypoints_left,
            matchings_msg.keypoints_right,
            matchings_msg.distances
        ):

            if distance >= 30:
                continue

            cantidad += 1

            # Punto izquierdo
            x_left = int(round(punto_left.x))
            y_left = int(round(punto_left.y))

            cv2.circle(imagen_left, (x_left, y_left), 3, (0, 255, 0), -1)

            # Punto derecho
            x_right = int(round(punto_right.x))
            y_right = int(round(punto_right.y))

            cv2.circle(imagen_right, (x_right, y_right), 3, (0, 255, 0), -1)

        cv2.imshow('Matchings izquierda < 30', imagen_left)
        cv2.imshow('Matchings derecha < 30', imagen_right)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = ViewMatchings()

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