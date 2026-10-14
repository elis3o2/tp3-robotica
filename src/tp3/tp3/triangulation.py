import cv2
import rclpy
import numpy as np
import struct

from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from sensor_msgs_py import point_cloud2

from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


from tp3_interfaces.msg import MatchingSet


class Triangulation(Node):

    def __init__(self):
        super().__init__('triangulation')

        self.matriz_left = None
        self.matriz_right = None

        self.bridge = CvBridge()

        # Información cámara izquierda
        self.sub_camera_left = self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_camara_left, 10
        )

        # Información cámara derecha
        self.sub_camera_right = self.create_subscription(
            CameraInfo, '/right/camera_info', self.procesar_camara_right, 10
        )

        # Imagen izquierda (para obtener color)
        self.sub_image_left = Subscriber(
            self, Image, '/left/image_rect'
        )

        # Matches normales
        self.sub_matching = Subscriber(
            self, MatchingSet, '/matchings'
        )

        self.sync_normal = ApproximateTimeSynchronizer(
            [self.sub_matching,
            self.sub_image_left ],
            queue_size=30,
            slop=0.02
        )

        self.sync_normal.registerCallback(
            self.procesar_normal
        )

        # Matches con RANSAC
        self.sub_matching_ransac = Subscriber(
            self, MatchingSet, '/ransac_matchings'
        )

        self.sync_ransac = ApproximateTimeSynchronizer(
            [self.sub_matching_ransac,
            self.sub_image_left],
            queue_size=30,
            slop=0.02
        )

        self.sync_ransac.registerCallback(
            self.procesar_ransac
        )

        # Nube 3D normal
        self.pub = self.create_publisher(
            PointCloud2, '/points3d', 10
        )

        # Nube 3D con RANSAC
        self.pub_ransac = self.create_publisher(
            PointCloud2, '/ransac_points3d', 10
        )

        self.get_logger().info(
            'Nodo de triangulación iniciado'
        )

    def procesar_camara_left(self, msg: CameraInfo):

        self.matriz_left = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        self.destroy_subscription(self.sub_camera_left)

        self.sub_camera_left = None

    def procesar_camara_right(self, msg: CameraInfo):

        self.matriz_right = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        self.destroy_subscription(self.sub_camera_right)

        self.sub_camera_right = None

    def puntos_a_array(self, puntos):

        return np.array(
            [ [p.x, p.y] for p in puntos],
            dtype=np.float64
        ).T

    def obtener_rgb(self, imagen, keypoint):

        u = int(round(keypoint.x))
        v = int(round(keypoint.y))

        alto, ancho = imagen.shape[:2]

        if u < 0 or u >= ancho or v < 0 or v >= alto:
            return None

        # La imagen se convierte a BGR8 con cv_bridge
        b, g, r = imagen[v, u]

        r = int(r)
        g = int(g)
        b = int(b)

        # Empaquetar:
        rgb_uint32 = ((r << 16) | (g << 8) | b )

        
        rgb_float = struct.unpack('f', struct.pack('I',rgb_uint32))[0]

        return rgb_float


    def procesar_matches(self, msg: MatchingSet, image_msg: Image):

        if self.matriz_left is None or self.matriz_right is None:
            return None

        if len(msg.keypoints_left) == 0:
            return None

        # Imagen izquierda correspondiente a estos matches
        try:
            imagen_left = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Error convirtiendo imagen: {e}'
            )

            return None
        
        puntos_left = self.puntos_a_array(msg.keypoints_left)
        puntos_right = self.puntos_a_array(msg.keypoints_right)

        
        # Triangulación
        puntos_4d = cv2.triangulatePoints(
            self.matriz_left,
            self.matriz_right,
            puntos_left,
            puntos_right
        )

        # Coordenadas homogéneas a coordenadas 3D
        puntos_3d = (puntos_4d[:3] / puntos_4d[3]).T

        puntos = []
        # Para cada punto:
        # x, y, z
        # index
        # rgb del pixel izquierdo
        for i, punto in enumerate(puntos_3d):

            # Descartar NaN / inf
            if not np.all(np.isfinite(punto)):
                continue

            keypoint_left = msg.keypoints_left[i]

            rgb = self.obtener_rgb(imagen_left, keypoint_left)

            if rgb is None:
                continue

            puntos.append([
                float(punto[0]),
                float(punto[1]),
                float(punto[2]),
                int(i),
                rgb
            ])

        if len(puntos) == 0:
            return None


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
                name='index',
                offset=12,
                datatype=PointField.UINT32,
                count=1
            ),

            PointField(
                name='rgb',
                offset=16,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        cloud = point_cloud2.create_cloud(
            msg.header,
            fields,
            puntos
        )

        return cloud

    def procesar_normal(self, msg: MatchingSet, image_msg: Image):

        cloud = self.procesar_matches(msg, image_msg)

        if cloud is not None:
            self.pub.publish(cloud)

    def procesar_ransac(self, msg: MatchingSet, image_msg: Image):

        cloud = self.procesar_matches(msg, image_msg)

        if cloud is not None:
            self.pub_ransac.publish(cloud)


def main(args=None):

    rclpy.init(args=args)

    node = Triangulation()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()