import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


class FeatureExtractor(Node):

    def __init__(self):
        super().__init__('feature_extractor')

        self.bridge = CvBridge()

        # Detector y descriptor
        self.orb = cv2.ORB_create(
            nfeatures=1000
        )

        # Matcher
        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=True
        )
        
        self.cam_info_left = None
        self.cam_info_right = None
        
        self.image_left = None
        self.image_right = None

        self.keypoints_left = None
        self.keypoints_right = None

        self.descriptors_left = None
        self.descriptors_right = None

        self.matches = None
        
        self.points3d = None

        self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_info_left, 10
        )

        self.create_subscription(
            CameraInfo, '/right/camera_info', self.procesar_info_right, 10
        )

        # Hacemos que izquierda y derecha esten sincronizados
        self.sub_left = Subscriber(
            self, Image, '/left/image_rect'
        )

        self.sub_right = Subscriber(
            self, Image, '/right/image_rect'
        )

        self.juntos = ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right],
            queue_size=10,
            slop=0.05
        )

        self.juntos.registerCallback(self.procesar)


        self.get_logger().info('Feature extractor iniciado')

    def procesar_info_left(self, msg):
        self.cam_info_left = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)


    def procesar_info_right(self, msg):
        self.cam_info_right = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)


    def extraer_features(self, msg):

        imagen = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='mono8'
        )

        keypoints, descriptors = self.orb.detectAndCompute(
            imagen,
            None
        )

        return imagen, keypoints, descriptors
    
    
    def triangular(self):

        if self.cam_info_left is None or self.cam_info_right is None:
            return None

        points_left = np.array([
            self.keypoints_left[m.queryIdx].pt
            for m in self.matches
        ])

        points_right = np.array([
            self.keypoints_right[m.trainIdx].pt
            for m in self.matches
        ])

        points_4d = cv2.triangulatePoints(
            self.cam_info_left,
            self.cam_info_right,
            points_left.T,
            points_right.T
        )

        points_3d = (
            points_4d[:3]
            / points_4d[3]
        ).T

        return points_3d
    
    def procesar(self, msg_left, msg_right):

        # Extraigo features izquierda
        (   self.image_left,
            self.keypoints_left,
            self.descriptors_left
        ) = self.extraer_features(msg_left)

        # Extraigo features derecha
        (   self.image_right,
            self.keypoints_right,
            self.descriptors_right
        ) = self.extraer_features(msg_right)


        # No se encuentran descriptores
        if ( self.descriptors_left is None
            or self.descriptors_right is None):
            
            return

        # Busco Matches
        self.matches = self.matcher.match(
            self.descriptors_left,
            self.descriptors_right
        )

        self.points3d = self.triangular()



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