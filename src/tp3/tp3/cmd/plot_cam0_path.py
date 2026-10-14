import cv2
import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tp3_interfaces.msg import FeatureSet


class PlotCam0Path(Node):

    def __init__(self):
        super().__init__('cam0_path')

        self.k = None

        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=True
        )

        # Frame anterior
        self.keypoints_anterior = None
        self.descriptors_anterior = None
        self.gt_anterior = None

        # Pose acumulada de la cámara
        self.R_global = np.eye(3)
        self.posicion_global = np.zeros(3)

        self.trayectoria = [self.posicion_global.copy()]

        # Info de la camara
        self.sub_camera = self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_camera_info, 10
        )

        # Features cámara izquierda
        self.sub_features = Subscriber(
            self, FeatureSet, '/left/features'
        )

        # Ground truth
        self.sub_gt = Subscriber(
            self, PointStamped, '/leica/position'
        )

        # Asociar cada frame con su posición ground-truth
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_features,
            self.sub_gt],
            queue_size=50,
            slop=0.05
        )

        self.sync.registerCallback(
            self.procesar
        )

        # Plot
        plt.ion()

        self.fig = plt.figure()

        self.ax = self.fig.add_subplot(
            111,
            projection='3d'
        )

        self.get_logger().info(
            'Plot cam0-path iniciado'
        )


    def procesar_camera_info(self, msg: CameraInfo):

        P = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        self.k = P[:, :3]

        self.destroy_subscription(
            self.sub_camera
        )

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


    def puntos_a_array(self, puntos):

        return np.array(
            [ [p.x, p.y] for p in puntos ],
            dtype=np.float64
        )


    def procesar(self, features_msg: FeatureSet, gt_msg: PointStamped):

        if self.k is None:
            return

        descriptors_actual = self.reconstruir_descriptores(features_msg)
        

        if descriptors_actual is None:
            return

        gt_actual = np.array([
            gt_msg.point.x,
            gt_msg.point.y,
            gt_msg.point.z
        ])

        # Primer frame: solamente guardarlo
        if self.descriptors_anterior is None:

            self.keypoints_anterior = features_msg.keypoints
            

            self.descriptors_anterior = descriptors_actual
            
            self.gt_anterior = gt_actual

            return


        # Matching frame anterior - actual
        matches = self.matcher.match(
            self.descriptors_anterior,
            descriptors_actual
        )

        if len(matches) < 5:
            self.guardar_actual(
                features_msg,
                descriptors_actual,
                gt_actual
            )
            return

        puntos_anterior = []
        puntos_actual = []

        for match in matches:

            puntos_anterior.append(self.keypoints_anterior[match.queryIdx])

            puntos_actual.append(features_msg.keypoints[match.trainIdx])

        puntos_anterior = self.puntos_a_array(puntos_anterior)

        puntos_actual = self.puntos_a_array(puntos_actual)

        
        # Matriz esencial
        E, mascara = cv2.findEssentialMat(
            puntos_anterior,
            puntos_actual,
            self.k,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if E is None:

            self.guardar_actual(
                features_msg,
                descriptors_actual,
                gt_actual
            )

            return

        if E.shape != (3, 3):
            E = E[:3, :]

        
        # Movimiento entre frames
        _, R, t, _ = cv2.recoverPose(
            E,
            puntos_anterior,
            puntos_actual,
            self.k,
            mask=mascara
        )

        # Distancia real entre ambos frames
        escala = np.linalg.norm(
            gt_actual - self.gt_anterior
        )

        # recoverPose devuelve:
        #
        # X_actual = R X_anterior + t
        #
        # La posición de la cámara actual
        # vista desde la cámara anterior es:
        desplazamiento = (-R.T @ t).flatten()

        desplazamiento *= escala

        # Llevar desplazamiento al sistema global
        self.posicion_global += self.R_global @ desplazamiento

        # Acumular orientación
        self.R_global = self.R_global @ R.T
        

        self.trayectoria.append(self.posicion_global.copy())

        self.graficar()

        self.guardar_actual(
            features_msg,
            descriptors_actual,
            gt_actual
        )


    def guardar_actual(self, features_msg, descriptors, gt):

        self.keypoints_anterior = features_msg.keypoints

        self.descriptors_anterior = descriptors

        self.gt_anterior = gt


    def graficar(self):

        trayectoria = np.array(self.trayectoria)

        self.ax.clear()

        self.ax.plot(
            trayectoria[:, 0],
            trayectoria[:, 1],
            trayectoria[:, 2]
        )

        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')

        self.ax.set_title(
            'Trayectoria estimada cámara izquierda'
        )

        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_zlim(-2, 5)

        self.ax.set_box_aspect([1, 1, 1])

        self.ax.view_init(
            elev=25,
            azim=45
        )


def main(args=None):

    rclpy.init(args=args)

    node = PlotCam0Path()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.graficar()

    plt.ioff()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()