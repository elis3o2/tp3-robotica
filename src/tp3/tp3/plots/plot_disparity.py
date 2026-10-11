import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PlotDisparity(Node):

    def __init__(self):
        super().__init__('plot_disparity')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/disparity', self.procesar, 10
        )

        self.get_logger().info(
            'Plot disparity iniciado'
        )


    def procesar(self, msg: Image):

        disparity = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='32FC1'
        )

        # Ignorar disparidades inválidas
        validos = disparity > 0

        if not np.any(validos):
            return

        # Normalizar solamente para visualizar
        disparity_vis = np.zeros_like(
            disparity,
            dtype=np.uint8
        )

        disparity_vis[validos] = cv2.normalize(
            disparity[validos],
            None,
            0,
            255,
            cv2.NORM_MINMAX
        ).flatten().astype(np.uint8)

        cv2.imshow(
            'Mapa de disparidad',
            disparity_vis
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PlotDisparity()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == '__main__':
    main()