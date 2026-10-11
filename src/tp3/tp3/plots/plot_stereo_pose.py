import rclpy
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from transforms3d.quaternions import quat2mat


class PlotStereoPose(Node):

    def __init__(self):
        super().__init__('plot_stereo_pose')

        self.sub = self.create_subscription(
            TransformStamped, '/stereo_transform_estimated', self.procesar, 10
        )

        plt.ion()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(
            111,
            projection='3d'
        )

        self.get_logger().info(
            'Plot stereo pose iniciado'
        )


    def procesar(self, msg: TransformStamped):

        # Traslación estimada left -> right
        t = np.array([
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z
        ])

        # Quaternion ROS: x y z w
        # transforms3d: w x y z
        q = [
            msg.transform.rotation.w,
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z
        ]

        R = quat2mat(q)

        # recoverPose da:
        # X_right = R X_left + t
        #
        # Para obtener la posición física de la cámara derecha
        # expresada en el sistema de la izquierda:
        R_right = R.T
        t_right = -R.T @ t

        self.ax.clear()

        # Cámara izquierda
        self.ax.scatter(
            0,
            0,
            0,
            s=80
        )

        self.ax.text(
            0,
            0,
            0,
            'Left'
        )

        # Cámara derecha
        self.ax.scatter(
            t_right[0],
            t_right[1],
            t_right[2],
            s=80
        )

        self.ax.text(
            t_right[0],
            t_right[1],
            t_right[2],
            'Right'
        )

        # Ejes de la cámara izquierda
        self.dibujar_ejes(
            np.zeros(3),
            np.eye(3)
        )

        # Ejes de la cámara derecha
        self.dibujar_ejes(
            t_right,
            R_right
        )

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.ax.set_title(
            'Pose estimada cámaras estéreo'
        )

        self.ax.set_box_aspect([1, 1, 1])

        plt.draw()
        plt.pause(0.001)


    def dibujar_ejes(self, origen, R, escala=0.05):

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


def main(args=None):

    rclpy.init(args=args)

    node = PlotStereoPose()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    plt.close('all')

    rclpy.shutdown()


if __name__ == '__main__':
    main()