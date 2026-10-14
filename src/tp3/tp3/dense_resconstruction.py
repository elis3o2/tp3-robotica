import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from cv_bridge import CvBridge
from tp3_interfaces.msg import MatrizTimeSet
from message_filters import Subscriber, TimeSynchronizer


class DenseReconstruction(Node):

    def __init__(self):
        super().__init__('dense_reconstruction')

        self.bridge = CvBridge()

        self.matriz_q = None
        self.matriz_p_left = None
        self.matriz_p_right = None

        # Información de las cámaras
        self.sub_camera_left = self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_camara_left, 10
        )

        self.sub_camera_right = self.create_subscription(
            CameraInfo, '/right/camera_info', self.procesar_camara_right, 10
        )

        # Imagen izquierda para obtener color
        self.sub_image = Subscriber(
            self, Image, '/left/image_rect'
        )

        # Mapa de disparidad
        self.sub_disparity = Subscriber(
            self, Image, '/disparity'
        )

        # /disparity conserva el timestamp de la imagen izquierda
        self.sync = TimeSynchronizer(
            [self.sub_image,
            self.sub_disparity],
            queue_size=30
        )

        self.sync.registerCallback(
            self.procesar
        )

        self.pub = self.create_publisher(
            PointCloud2, '/dense_points3d', 10
        )

        self.pub_matriz = self.create_publisher(
            MatrizTimeSet, '/q_matriz', 10
        )
        
        self.paso = 2

        self.get_logger().info(
            'Reconstrucción densa iniciada'
        )


    def procesar_camara_left(self, msg: CameraInfo):

        self.matriz_p_left = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        self.calcular_q()


    def procesar_camara_right(self, msg: CameraInfo):

        self.matriz_p_right = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        self.calcular_q()


    def calcular_q(self):

        if self.matriz_p_left is None or self.matriz_p_right is None:
            return

        fx = self.matriz_p_left[0, 0]

        cx_left = self.matriz_p_left[0, 2]
        cy_left = self.matriz_p_left[1, 2]

        cx_right = self.matriz_p_right[0, 2]

        # P_right[0, 3] = fx_right * Tx
        tx = self.matriz_p_right[0, 3] / self.matriz_p_right[0, 0]

        if tx == 0:
            self.get_logger().error(
                'No se puede calcular Q: baseline igual a 0'
            )
            return

        self.matriz_q = np.array([
            [1.0, 0.0, 0.0, -cx_left],
            [0.0, 1.0, 0.0, -cy_left],
            [0.0, 0.0, 0.0, fx],
            [0.0, 0.0, -1.0 / tx, (cx_left - cx_right) / tx]
        ], dtype=np.float64)


        if self.sub_camera_left is not None:
            self.destroy_subscription(
                self.sub_camera_left
            )
            self.sub_camera_left = None

        if self.sub_camera_right is not None:
            self.destroy_subscription(
                self.sub_camera_right
            )
            self.sub_camera_right = None

    def procesar(self, image_msg: Image, disparity_msg: Image):

        if self.matriz_q is None:
            return

        imagen = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8'
        )

        disparity = self.bridge.imgmsg_to_cv2(
            disparity_msg,
            desired_encoding='32FC1'
        )

        # Reproyectar todos los píxeles al espacio 3D
        puntos_3d = cv2.reprojectImageTo3D(disparity, self.matriz_q)

        puntos = []

        alto, ancho = disparity.shape

        for y in range(0, alto, self.paso):

            for x in range(0, ancho, self.paso):

                # Disparidad inválida
                if disparity[y, x] <= 0:
                    continue

                X, Y, Z = puntos_3d[y, x]

                if not np.all(np.isfinite([X, Y, Z])):
                    continue

                # OpenCV usa BGR
                b, g, r = imagen[y, x]

                rgb = ((int(r) << 16) | (int(g) << 8) | int(b))

                puntos.append([
                    float(X),
                    float(Y),
                    float(Z),
                    rgb
                ])

        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='rgb',
                offset=12,
                datatype=PointField.UINT32,
                count=1
            )
        ]

        cloud = point_cloud2.create_cloud(
            disparity_msg.header,
            fields,
            puntos
        )

        self.pub.publish(cloud)
        
        # Publicar matriz q
        matriz_msg = MatrizTimeSet()

        matriz_msg.header = image_msg.header

        matriz_msg.matriz = (self.matriz_q.flatten().tolist())

        self.pub_matriz.publish(matriz_msg)


def main(args=None):

    rclpy.init(args=args)

    node = DenseReconstruction()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()