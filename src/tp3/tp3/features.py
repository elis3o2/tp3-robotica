import cv2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge

from tp3_interfaces.msg import FeatureSet

class FeatureExtractor(Node):

    def __init__(self):
        super().__init__('feature_extractor')

        self.bridge = CvBridge()

        self.orb = cv2.ORB_create(
            nfeatures=1000
        )

        # Suscribo a imágen izquierda
        self.sub_left = self.create_subscription(
            Image, '/left/image_rect', self.procesar_left, 10
        )

        # Suscribo a imágen derecha
        self.sub_right = self.create_subscription(
            Image, '/right/image_rect', self.procesar_right, 10
        )

        # Publico features izquierda
        self.pub_left = self.create_publisher(
            FeatureSet, '/left/features', 10
        )

        # Publico features derecha 
        self.pub_right = self.create_publisher(
            FeatureSet, '/right/features', 10
        )

        self.get_logger().info('Feature extractor iniciado')


    def extraer_features(self, image_msg):

        # Convierto a OpenCV
        imagen = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='mono8'
        )

        # Obtengo keypoints y descriptors
        keypoints, descriptors = self.orb.detectAndCompute(
            imagen,
            None
        )

        # Creo el mensaje a publicar
        msg = FeatureSet()

        # Me quedo con el timestamp de la imagen
        msg.header = image_msg.header

        # Keypoints
        for kp in keypoints:

            punto = Point32()

            punto.x = float(kp.pt[0])
            punto.y = float(kp.pt[1])
            punto.z = 0.0

            msg.keypoints.append(punto)

        # Descriptors
        if descriptors is not None:

            msg.descriptor_size = descriptors.shape[1]

            msg.descriptors = (
                descriptors
                .flatten()
                .tolist()
            )

        return msg
    

    def procesar_left(self, image_msg):

        features_msg = self.extraer_features(image_msg)

        self.pub_left.publish(features_msg)


    def procesar_right(self, image_msg):

        features_msg = self.extraer_features(image_msg)

        self.pub_right.publish(features_msg)


def main(args=None):

    rclpy.init(args=args)
    node = FeatureExtractor()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()