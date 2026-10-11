import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, TimeSynchronizer

from tp3_interfaces.msg import MatchingSet, MatrizTimeSet


class PlotEpilines(Node):

    def __init__(self):
        super().__init__('plot_epilines')

        self.bridge = CvBridge()

        self.sub_image = Subscriber(
            self, Image, '/left/image_rect'
        )

        self.sub_matchings = Subscriber(
            self, MatchingSet, '/ransac_matchings'
        )

        self.sub_f = Subscriber(
            self, MatrizTimeSet, '/f_matriz'
        )

        self.sync = TimeSynchronizer(
            [ self.sub_image,
            self.sub_matchings,
            self.sub_f ],
            queue_size=50
        )

        self.sync.registerCallback(
            self.procesar
        )


    def procesar(self, image_msg, matchings_msg, matriz_msg):

        if len(matchings_msg.keypoints_right) == 0:
            return

        imagen = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8'
        )

        # Reconstruir matriz fundamental
        F = np.array(
            matriz_msg.matriz_f,
            dtype=np.float64
        ).reshape(3, 3)

        # Puntos de la imagen derecha
        puntos_right = np.array(
            [[p.x, p.y] for p in matchings_msg.keypoints_right],
            dtype=np.float32
        ).reshape(-1, 1, 2)

        # Líneas epipolares correspondientes
        # en la imagen izquierda
        lineas = cv2.computeCorrespondEpilines(puntos_right, 2, F).reshape(-1, 3)

        alto, ancho = imagen.shape[:2]

        for linea, punto_left in zip(lineas, matchings_msg.keypoints_left):

            a, b, c = linea
            # ax + by + c = 0
            if abs(b) > 1e-6:

                x1 = 0
                y1 = int(-c / b)

                x2 = ancho - 1
                y2 = int(-(c + a * x2) / b)

                cv2.line(
                    imagen,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    1
                )

            # Punto correspondiente izquierdo
            x = int(round(punto_left.x))
            y = int(round(punto_left.y))

            cv2.circle(
                imagen,
                (x, y),
                4,
                (0, 0, 255),
                -1
            )

        cv2.imshow(
            'Lineas epipolares',
            imagen
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PlotEpilines()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == '__main__':
    main()