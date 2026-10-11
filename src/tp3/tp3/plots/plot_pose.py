import cv2
import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

from sensor_msgs.msg import CameraInfo

from message_filters import Subscriber, TimeSynchronizer

from tp3_interfaces.msg import FeatureSet, MatchingSet



class PnPPose(Node):

    def __init__(self):
        super().__init__('pnp_pose')

        self.P_left = None
        self.P_right = None
        self.K = None

        self.inicializado = False
        self.terminado = False

        self.t_inicial = None

        self.puntos_3d = None
        self.descriptores_iniciales = None

        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=True
        )

        # CameraInfo
        self.sub_camera_left = self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_camara_left, 10
        )

        self.sub_camera_right = self.create_subscription(
            CameraInfo, '/right/camera_info', self.procesar_camara_right, 10
        )

        # Frame inicial
        self.sub_ransac = Subscriber(
            self, MatchingSet, '/ransac_matchings'
        )

        self.sub_features_inicial = Subscriber(
            self, FeatureSet, '/left/features'
        )

        self.sync = TimeSynchronizer(
            [self.sub_ransac,
            self.sub_features_inicial],
            queue_size=50
        )

        self.sync.registerCallback(
            self.procesar_inicial
        )

        # Frames futuros
        self.sub_features = self.create_subscription(
            FeatureSet,
            '/left/features',
            self.procesar_frame,
            10
        )

        self.get_logger().info(
            'Nodo PnP iniciado'
        )


    def procesar_camara_left(self, msg):

        self.P_left = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        # Estamos usando imágenes rectificadas
        self.K = self.P_left[:, :3]

        self.destroy_subscription(self.sub_camera_left)

        self.sub_camera_left = None


    def procesar_camara_right(self, msg):

        self.P_right = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        self.destroy_subscription(self.sub_camera_right)

        self.sub_camera_right = None


    def reconstruir_descriptores(self, msg):

        if len(msg.descriptors) == 0:
            return None

        return np.array(
            msg.descriptors,
            dtype=np.uint8
        ).reshape(
            len(msg.keypoints),
            msg.descriptor_size
        )


    def puntos_a_array(self, puntos):

        return np.array(
            [[p.x, p.y] for p in puntos ],
            dtype=np.float64
        )


    def stamp_segundos(self, stamp):

        return stamp.sec + stamp.nanosec * 1e-9


    def procesar_inicial(self, matches_msg, features_msg):

        if self.inicializado:
            return

        if (
            self.P_left is None
            or self.P_right is None
            or self.K is None
        ):
            return

        if len(matches_msg.keypoints_left) < 4:
            return

        descriptors = self.reconstruir_descriptores(features_msg)

        if descriptors is None:
            return

        # Triangulación

        puntos_left = self.puntos_a_array(matches_msg.keypoints_left)

        puntos_right = self.puntos_a_array(matches_msg.keypoints_right)

        puntos_4d = cv2.triangulatePoints(
            self.P_left,
            self.P_right,
            puntos_left.T,
            puntos_right.T
        )

        puntos_3d = (puntos_4d[:3] / puntos_4d[3]).T

        
        # Relacionar cada punto 3D con su descriptor izquierdo
        indice_features = {}

        for i, punto in enumerate(features_msg.keypoints):

            clave = (round(punto.x, 4), round(punto.y, 4))

            indice_features[clave] = i

        puntos_validos = []
        descriptors_validos = []

        for punto_3d, punto_2d in zip(puntos_3d, matches_msg.keypoints_left):

            if not np.all(np.isfinite(punto_3d)):
                continue

            # Punto detrás de la cámara
            if punto_3d[2] <= 0:
                continue

            clave = (round(punto_2d.x, 4), round(punto_2d.y, 4))

            if clave not in indice_features:
                continue

            indice = indice_features[clave]

            puntos_validos.append(punto_3d)

            descriptors_validos.append(descriptors[indice])

        if len(puntos_validos) < 4:
            return

        self.puntos_3d = np.array(
            puntos_validos,
            dtype=np.float32
        )

        self.descriptores_iniciales = np.array(
            descriptors_validos,
            dtype=np.uint8
        )

        self.t_inicial = self.stamp_segundos(
            features_msg.header.stamp
        )

        self.inicializado = True


    def procesar_frame(self, msg):

        if not self.inicializado:
            return

        if self.terminado:
            return

        t_actual = self.stamp_segundos(msg.header.stamp)

        diferencia = t_actual - self.t_inicial
        

        # Queremos aproximadamente un segundo después
        if diferencia < 1.0:
            return

        descriptors_actual = self.reconstruir_descriptores(msg)

        if descriptors_actual is None:
            return

        # -------------------------
        # Matching temporal
        # -------------------------

        matches = self.matcher.match(
            self.descriptores_iniciales,
            descriptors_actual
        )

        if len(matches) < 4:
            return

        puntos_3d = []
        puntos_2d = []

        for match in matches:

            puntos_3d.append(
                self.puntos_3d[
                    match.queryIdx
                ]
            )

            punto = msg.keypoints[
                match.trainIdx
            ]

            puntos_2d.append([
                punto.x,
                punto.y
            ])

        puntos_3d = np.array(
            puntos_3d,
            dtype=np.float32
        )

        puntos_2d = np.array(
            puntos_2d,
            dtype=np.float32
        )

        # -------------------------
        # PnP + RANSAC
        # -------------------------
        exito, rvec, tvec, inliers = (
            cv2.solvePnPRansac(
                puntos_3d,
                puntos_2d,
                self.K,
                None,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        )

        if not exito:
            self.get_logger().warning(
                'solvePnPRansac no encontró una pose'
            )
            return

        self.terminado = True

        self.get_logger().info(
            f'Pose estimada a {diferencia:.3f} s'
        )

        if inliers is not None:
            self.get_logger().info(
                f'Inliers PnP: '
                f'{len(inliers)}/{len(matches)}'
            )

        # Rotación world -> camera
        R, _ = cv2.Rodrigues(rvec)

        # solvePnP devuelve:
        #
        # X_camera = R X_world + t
        #
        # Entonces la posición de la cámara
        # en coordenadas del frame inicial es:
        posicion_camara = (
            -R.T @ tvec
        ).flatten()

        # Orientación camera -> world
        R_camara = R.T

        self.get_logger().info(
            f'Posicion estimada: '
            f'{posicion_camara}'
        )

        self.graficar(
            posicion_camara,
            R_camara
        )


    def dibujar_camara(
        self,
        origen,
        R,
        nombre,
        escala=0.2
    ):

        self.ax.scatter(
            origen[0],
            origen[1],
            origen[2],
            s=80
        )

        self.ax.text(
            origen[0],
            origen[1],
            origen[2],
            nombre
        )

        for i in range(3):

            direccion = (
                R[:, i] * escala
            )

            self.ax.quiver(
                origen[0],
                origen[1],
                origen[2],
                direccion[0],
                direccion[1],
                direccion[2]
            )


    def graficar(
        self,
        posicion_camara,
        R_camara
    ):

        plt.figure()

        self.ax = plt.axes(
            projection='3d'
        )

        # Puntos triangulados
        self.ax.scatter(
            self.puntos_3d[:, 0],
            self.puntos_3d[:, 1],
            self.puntos_3d[:, 2],
            s=5
        )

        # Pose inicial
        self.dibujar_camara(
            np.zeros(3),
            np.eye(3),
            'Cam t'
        )

        # Pose estimada un segundo después
        self.dibujar_camara(
            posicion_camara,
            R_camara,
            'Cam t+1'
        )

        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')

        self.ax.set_title(
            'Pose estimada con solvePnPRansac'
        )

        plt.show()


def main(args=None):

    rclpy.init(args=args)

    node = PnPPose()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    plt.close('all')

    rclpy.shutdown()


if __name__ == '__main__':
    main()