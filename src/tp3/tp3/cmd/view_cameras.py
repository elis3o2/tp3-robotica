import cv2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ViewCameras(Node):

    def __init__(self):
        super().__init__('plot_cameras')

        self.bridge = CvBridge()

        self.sub_left_raw = self.create_subscription(
            Image, '/left/image_raw', self.procesar_left_raw, 10
        )

        self.sub_right_raw = self.create_subscription(
            Image, '/right/image_raw', self.procesar_right_raw, 10
        )

        self.sub_left_rect = self.create_subscription(
            Image, '/left/image_rect', self.procesar_left_rect, 10
        )

        self.sub_right_rect = self.create_subscription(
            Image, '/right/image_rect', self.procesar_right_rect, 10
        )

        self.get_logger().info(
            'View de cámaras iniciado'
        )


    def mostrar(self, nombre, msg):

        imagen = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Reducir para que las cuatro ventanas no sean tan grandes
        imagen = cv2.resize(
            imagen,
            None,
            fx=0.5,
            fy=0.5
        )

        cv2.imshow(nombre, imagen)
        cv2.waitKey(1)


    def procesar_left_raw(self, msg):
        self.mostrar('Left Raw', msg)


    def procesar_right_raw(self, msg):
        self.mostrar('Right Raw', msg)


    def procesar_left_rect(self, msg):
        self.mostrar('Left Rectified', msg)


    def procesar_right_rect(self, msg):
        self.mostrar('Right Rectified', msg)


def main(args=None):

    rclpy.init(args=args)

    node = ViewCameras()

    try:

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

            cv2.waitKey(1)
            
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == '__main__':
    main()