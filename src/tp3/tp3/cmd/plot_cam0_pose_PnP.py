import cv2
import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2

from message_filters import Subscriber, TimeSynchronizer

from tp3_interfaces.msg import FeatureSet, MatchingSet


class PnPPose(Node):

    def __init__(self):
        super().__init__('pnp_pose')

        self.K = None

        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=True
        )

        # Frame que se toma como referencia
        self.referencia = None

        # Cuando obtenemos una pose terminamos
        self.terminado = False

        # CameraInfo
        self.sub_camera = self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_camera_info, 10
        )

        # Nube 3D triangulada 
        self.sub_cloud = Subscriber(
            self, PointCloud2, '/ransac_points3d'
        )

        # Matches usados para generar la nube
        self.sub_ransac = Subscriber(
            self, MatchingSet, '/ransac_matchings'
        )

        # Features de la cámara izquierda
        self.sub_features = Subscriber(
            self, FeatureSet, '/left/features'
        )

        # Los tres mensajes pertenecen al mismo frame
        self.sync = TimeSynchronizer(
            [self.sub_cloud,
            self.sub_ransac,
            self.sub_features],
            queue_size=50
        )

        self.sync.registerCallback(
            self.procesar
        )

        self.get_logger().info(
            'Estimación de pose PnP iniciada'
        )


    def procesar_camera_info(self, msg: CameraInfo):

        P = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        # Intrínseca de la cámara rectificada
        self.K = P[:, :3]

        self.get_logger().info(
            f'Matriz K:\n{self.K}'
        )

        self.destroy_subscription(self.sub_camera)

        self.sub_camera = None


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


    def stamp_segundos(self, stamp):

        return stamp.sec + stamp.nanosec * 1e-9


    def crear_referencia(self, cloud_msg, matches_msg, features_msg):

        descriptors = self.reconstruir_descriptores(features_msg)

        if descriptors is None:
            return None

        # Relacionar posición del keypoint
        # con su descriptor original
        indice_features = {}

        for i, punto in enumerate(features_msg.keypoints):

            clave = (round(punto.x, 4), round(punto.y, 4))

            indice_features[clave] = i

        # Leer puntos de la nube 3D
        puntos_cloud = point_cloud2.read_points_list(
            cloud_msg,
            field_names=(
                'x',
                'y',
                'z',
                'index'
            ),
            skip_nans=True
        )

        puntos_3d = []
        descriptors_3d = []

        for punto in puntos_cloud:

            x = float(punto.x)
            y = float(punto.y)
            z = float(punto.z)

            indice_match = int(punto.index)

            if indice_match >= len(matches_msg.keypoints_left):
                continue

            # Punto detrás de la cámara
            if z <= 0:
                continue

            # Keypoint que originó este punto 3D
            punto_2d = matches_msg.keypoints_left[indice_match]
            

            clave = (round(punto_2d.x, 4), round(punto_2d.y, 4))

            if clave not in indice_features:
                continue

            indice_feature = (
                indice_features[clave]
            )

            puntos_3d.append([x, y, z])

            descriptors_3d.append(descriptors[indice_feature])

        if len(puntos_3d) < 4:
            return None

        return {
            'tiempo': self.stamp_segundos(
                features_msg.header.stamp
            ),

            'puntos_3d': np.array(
                puntos_3d,
                dtype=np.float32
            ),

            'descriptores': np.array(
                descriptors_3d,
                dtype=np.uint8
            )
        }


    def procesar(self, cloud_msg, matches_msg, features_msg):

        if self.terminado:
            return

        if self.K is None:
            return

        referencia_actual = (
            self.crear_referencia(
                cloud_msg,
                matches_msg,
                features_msg
            )
        )

        if referencia_actual is None:
            return

        # Primer frame válido:
        # se utiliza como origen
        if self.referencia is None:

            self.referencia = referencia_actual

            self.get_logger().info(
                'Primera referencia guardada'
            )

            return

        diferencia = referencia_actual['tiempo'] - self.referencia['tiempo']


        # Queremos un segundo entre las poses
        if diferencia < 1.0:
            return

        descriptors_actual = self.reconstruir_descriptores(features_msg)
        

        if descriptors_actual is None:
            return

        # Matching entre frame referencia y frame actual
        matches = self.matcher.match(
            self.referencia[
                'descriptores'
            ],
            descriptors_actual
        )

        if len(matches) < 4:

            self.get_logger().warning(
                'No hay suficientes matches temporales'
            )

            return

        puntos_3d = []
        puntos_2d = []

        for match in matches:

            # Punto 3D expresado en el sistema de la cámara referencia
            punto_3d = (
                self.referencia[
                    'puntos_3d'
                ][match.queryIdx]
            )

            # Posición del mismo feature en la imagen actual
            punto_2d = (
                features_msg.keypoints[
                    match.trainIdx
                ]
            )

            puntos_3d.append(punto_3d)

            puntos_2d.append([
                punto_2d.x,
                punto_2d.y
            ])

        puntos_3d = np.array(
            puntos_3d,
            dtype=np.float32
        )

        puntos_2d = np.array(
            puntos_2d,
            dtype=np.float32
        )

        # Estimar transformación mediante PnP
        exito, rvec, tvec, inliers = (
            cv2.solvePnPRansac(
                puntos_3d,
                puntos_2d,
                self.K,
                None,
                iterationsCount=100,
                reprojectionError=3.0,
                confidence=0.99,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        )

        if not exito:

            self.get_logger().warning(
                'PnP falló'
            )

            return

        # Matriz de rotación
        R, _ = cv2.Rodrigues(
            rvec
        )

        # solvePnP devuelve:
        #
        # X_actual =
        # R X_referencia + t
        #
        # Esta transformación expresa
        # puntos del frame referencia
        # en el sistema del frame actual.
        #
        # Para obtener la pose física
        # de la cámara actual respecto
        # de la cámara referencia,
        # invertimos la transformación.

        R_relativa = R.T

        posicion_relativa = (-R.T @ tvec).flatten()

        cantidad_inliers = len(inliers) if inliers is not None else 0

        self.get_logger().info(
            f'Pose estimada: '
            f't={diferencia:.2f}s '
            f'inliers={cantidad_inliers}/'
            f'{len(matches)} '
            f'pos={posicion_relativa}'
        )

        # Mostrar  estas dos poses
        self.graficar(
            posicion_relativa,
            R_relativa
        )

        self.terminado = True


    def dibujar_camara(self, ax, posicion, R, escala=0.15):

        for i in range(3):

            direccion = R[:, i] * escala

            ax.quiver(
                posicion[0],
                posicion[1],
                posicion[2],
                direccion[0],
                direccion[1],
                direccion[2]
            )


    def graficar(self, posicion, R):

        fig = plt.figure()

        ax = fig.add_subplot(
            111,
            projection='3d'
        )

        # Cámara izquierda en el instante de referencia
        origen = np.zeros(3)

        ax.scatter(
            origen[0],
            origen[1],
            origen[2],
            s=70
        )

        ax.text(
            origen[0],
            origen[1],
            origen[2],
            'Referencia'
        )

        self.dibujar_camara(
            ax,
            origen,
            np.eye(3)
        )

        # Cámara izquierda  1 segundo después
        ax.scatter(
            posicion[0],
            posicion[1],
            posicion[2],
            s=70
        )

        ax.text(
            posicion[0],
            posicion[1],
            posicion[2],
            'Actual'
        )

        self.dibujar_camara(
            ax,
            posicion,
            R
        )

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')

        ax.set_title(
            'Pose estimada de la cámara izquierda - PnP'
        )

        ax.set_box_aspect(
            [1, 1, 1]
        )


def main(args=None):

    rclpy.init(args=args)

    node = PnPPose()

    # Procesar hasta obtener
    # una estimación válida
    while (
        rclpy.ok()
        and not node.terminado
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )

    node.destroy_node()

    rclpy.shutdown()

    # Mostrar solamente la estimación obtenida
    plt.show()


if __name__ == '__main__':
    main()