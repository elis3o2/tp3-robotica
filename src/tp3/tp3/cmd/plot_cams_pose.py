import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from transforms3d.quaternions import quat2mat


class PlotCamsPose(Node):

    def __init__(self):
        super().__init__('cams_pose')

        self.terminado = False
        
        self.sub = self.create_subscription(
            TransformStamped, '/transformation_estimated', self.procesar, 10
        )

        self.graficado = False

        self.get_logger().info(
            'Esperando una estimación de pose estéreo...'
        )


    def procesar(self, msg: TransformStamped):

        # Graficar solamente el primer mensaje
        if self.graficado:
            return

        self.graficado = True

        # Transformación estimada por recoverPose
        # X_right = R X_left + t

        t = np.array([
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z
        ])

        # ROS: x y z w
        # transforms3d: w x y z
        q = [
            msg.transform.rotation.w,
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z
        ]

        R = quat2mat(q)

        # Matriz homogénea left -> right
        T_left_right = np.eye(4)

        T_left_right[:3, :3] = R
        T_left_right[:3, 3] = t

        print("\nMatriz estimada Left -> Right:")
        print(T_left_right)

        # recoverPose devuelve:
        #
        # X_right = R X_left + t
        #
        # Para visualizar físicamente la cámara derecha
        # en el sistema de coordenadas de la izquierda:

        R_right = R.T
        t_right = -R.T @ t

        T_right_pose = np.eye(4)

        T_right_pose[:3, :3] = R_right
        T_right_pose[:3, 3] = t_right

        print("\nPose de la cámara derecha respecto de la izquierda:")
        print(T_right_pose)

        self.graficar(t_right, R_right)

        self.terminado = True


    def graficar(self, t_right, R_right):

        fig = plt.figure()

        ax = fig.add_subplot(
            111,
            projection='3d'
        )

        # Cámara izquierda
        ax.scatter(0, 0, 0, s=80)

        ax.text(
            0,
            0,
            0,
            'Left'
        )

        # Cámara derecha
        ax.scatter(
            t_right[0],
            t_right[1],
            t_right[2],
            s=80
        )

        ax.text(
            t_right[0],
            t_right[1],
            t_right[2],
            'Right'
        )

        # Ejes cámara izquierda
        self.dibujar_ejes(
            ax,
            np.zeros(3),
            np.eye(3)
        )

        # Ejes cámara derecha
        self.dibujar_ejes(
            ax,
            t_right,
            R_right
        )

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')

        ax.set_title('Pose estimada de las cámaras')

        ax.set_box_aspect([1, 1, 1])



    def dibujar_ejes(self, ax, origen, R, escala=0.05):

        for i in range(3):

            direccion = R[:, i] * escala

            ax.quiver(
                origen[0],
                origen[1],
                origen[2],
                direccion[0],
                direccion[1],
                direccion[2]
            )


def main(args=None):

    rclpy.init(args=args)

    node = PlotCamsPose()

    while rclpy.ok() and not node.terminado:
        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )

    node.destroy_node()
    rclpy.shutdown()

    plt.show()


if __name__ == '__main__':
    main()
