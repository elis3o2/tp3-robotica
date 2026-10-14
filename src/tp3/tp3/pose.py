import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped

from tp3_interfaces.msg import MatchingSet, MatrizTimeSet

from transforms3d.quaternions import mat2quat


class Pose(Node):

    def __init__(self):
        super().__init__('pose')

        self.k = None
        self.baseline = None

        self.frame_left = ''
        self.frame_right = ''

        # CameraInfo izquierda
        self.sub_camera_left = self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_camara_left, 10
        )

        # CameraInfo derecha
        self.sub_camera_right = self.create_subscription(
            CameraInfo, '/right/camera_info', self.procesar_camara_right, 10
        )

        # Matches ya filtrados por RANSAC
        self.sub_matches = self.create_subscription(
            MatchingSet, '/ransac_matchings', self.procesar_matches, 10
        )

        # Transformación estimada
        self.pub = self.create_publisher(
            TransformStamped, '/transformation_estimated', 10
        )

        self.pub_matriz = self.create_publisher(
            MatrizTimeSet, '/e_matriz', 10
        )
        
        self.get_logger().info(
            'Nodo stereo_pose iniciado'
        )


    def procesar_camara_left(self, msg: CameraInfo):

        P = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        # Como trabajamos con imágenes rectificadas,
        # usamos la intrínseca contenida en P
        self.k = P[:, :3]

        self.frame_left = msg.header.frame_id

        self.destroy_subscription(self.sub_camera_left)

        self.sub_camera_left = None


    def procesar_camara_right(self, msg: CameraInfo):

        P = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)

        fx = P[0, 0]


        # P_right[0,3] = fx * Tx
        # El baseline es |Tx|
        self.baseline = abs(P[0, 3] / fx)

        self.frame_right = msg.header.frame_id

        self.get_logger().info(
            f'Baseline: {self.baseline:.4f} m'
        )

        self.destroy_subscription(self.sub_camera_right)

        self.sub_camera_right = None


    def puntos_a_array(self, puntos):

        return np.array(
            [[p.x, p.y] for p in puntos],
            dtype=np.float64
        )


    def procesar_matches(self, msg: MatchingSet):

        if self.k is None or self.baseline is None:
            return

        # No se puede calcular
        if len(msg.keypoints_left) < 5:
            return

        puntos_left = self.puntos_a_array(msg.keypoints_left)

        puntos_right = self.puntos_a_array(msg.keypoints_right)

        # Matriz esencial
        E, mascara = cv2.findEssentialMat(
            puntos_left,
            puntos_right,
            self.k,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if E is None:
            return

        # Por seguridad si OpenCV devuelve varias soluciones
        if E.shape != (3, 3):
            E = E[:3, :]

        # Recuperar rotación y dirección de traslación
        _, R, t, _ = cv2.recoverPose(
            E,
            puntos_left,
            puntos_right,
            self.k,
            mask=mascara
        )

        # recoverPose devuelve ||t|| = 1
        # Le damos escala real usando el baseline
        t = t * self.baseline

        qw, qx, qy, qz = mat2quat(R)

        transform = TransformStamped()

        transform.header = msg.header
        transform.header.frame_id = self.frame_left

        transform.child_frame_id = self.frame_right

        transform.transform.translation.x = float(t[0, 0])
        transform.transform.translation.y = float(t[1, 0])
        transform.transform.translation.z = float(t[2, 0])

        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)

        self.pub.publish(
            transform
        )
        
        # Publicar matriz e
        matriz_msg = MatrizTimeSet()

        matriz_msg.header = msg.header

        matriz_msg.matriz = (E.flatten().tolist())

        self.pub_matriz.publish(matriz_msg)


def main(args=None):

    rclpy.init(args=args)

    node = Pose()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()