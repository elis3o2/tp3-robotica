import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tp3_interfaces.msg import FeatureSet, MatchingSet


class MatchingExtractor(Node):

    def __init__(self):
        super().__init__('matching_extractor')

        # Matcher
        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=True
        )

        # Suscribo  a features de cámara izquierda
        self.sub_left = Subscriber(
            self, FeatureSet, '/left/features'
        )

        # Suscribo a Features de cámara derecha
        self.sub_right = Subscriber(
            self, FeatureSet, '/right/features'
        )

        # Sincronizamos según header.stamp
        self.juntos = ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right],
            queue_size=10,
            slop=0.05
        )

        self.juntos.registerCallback(self.procesar)

        # Publisher de los matches
        self.pub = self.create_publisher(
            MatchingSet, '/matchings', 10
        )

        self.get_logger().info('Matching extractor iniciado')


    def procesar(self, msg_left: FeatureSet, msg_right: FeatureSet):


        if (
            len(msg_left.keypoints) == 0
            or len(msg_right.keypoints) == 0
            or len(msg_left.descriptors) == 0
            or len(msg_right.descriptors) == 0
        ):
            self.get_logger().info('No hay features')
            return


        # Reconstruyo descriptors
        descriptors_left = np.array(
            msg_left.descriptors,
            dtype=np.uint8
        ).reshape(
            len(msg_left.keypoints),
            msg_left.descriptor_size
        )

        descriptors_right = np.array(
            msg_right.descriptors,
            dtype=np.uint8
        ).reshape(
            len(msg_right.keypoints),
            msg_right.descriptor_size
        )
        

        # Busco matches
        matches = self.matcher.match(
            descriptors_left,
            descriptors_right
        )
        
        msg = MatchingSet()

        # Queda con el timestamp de la izquierda
        msg.header = msg_left.header

        for match in matches:

            # Punto izquierdo correspondiente al descriptor queryIdx
            msg.keypoints_left.append(
                msg_left.keypoints[match.queryIdx]
            )

            # Punto derecho correspondiente al descriptor trainIdx
            msg.keypoints_right.append(
                msg_right.keypoints[match.trainIdx]
            )

            # Distancia del match
            msg.distances.append(
                float(match.distance)
            )

        self.pub.publish(msg)



def main(args=None):

    rclpy.init(args=args)

    node = MatchingExtractor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()