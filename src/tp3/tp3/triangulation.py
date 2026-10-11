import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2



from tp3_interfaces.msg import MatchingSet


class Triangulation(Node):

    def __init__(self):
        super().__init__('triangulation')

        self.matriz_left = None 
        self.matriz_right = None 
        
        # Veo la informacion de la camara izquierda
        self.sub_camera_left = self.create_subscription(
            CameraInfo, '/left/camera_info', self.procesar_camara_left, 10
        )

        # Veo la informacion de la camara derecha
        self.sub_camera_right = self.create_subscription(
            CameraInfo, '/right/camera_info', self.procesar_camara_right, 10
        )

        # Matches
        self.sub_matching = self.create_subscription(
            MatchingSet, '/matchings', self.procesar_normal, 10
        )

        # Matches de RANSAC
        self.sub_matching = self.create_subscription(
            MatchingSet, '/ransac_matchings', self.procesar_ransac, 10
        )
        
        # Nube de puntos 3D
        self.pub = self.create_publisher(
            PointCloud2, '/points3d', 10
        )

        self.pub_ransac = self.create_publisher(
            PointCloud2, '/ransac_points3d', 10
        )
        
        self.get_logger().info(
            'Nodo de triangulación iniciado'
        )


    def procesar_camara_left(self, msg: CameraInfo):

        self.matriz_left = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)
        
        self.destroy_subscription(
            self.sub_camera_left
        )

    def procesar_camara_right(self, msg: CameraInfo):

        self.matriz_right = np.array(
            msg.p,
            dtype=np.float64
        ).reshape(3, 4)
        
        self.destroy_subscription(
            self.sub_camera_right
        )
                
    def puntos_a_array(self, puntos):
        return np.array(
            [[p.x, p.y] for p in puntos],
            dtype=np.float64
        ).T


    def procesar_matches(self, msg: MatchingSet):

        if self.matriz_left is None or self.matriz_right is None:
            return

        if len(msg.keypoints_left) == 0:
            return

        puntos_left = self.puntos_a_array(msg.keypoints_left)
        puntos_right = self.puntos_a_array(msg.keypoints_right)


        puntos_4d = cv2.triangulatePoints(
            self.matriz_left,
            self.matriz_right,
            puntos_left,
            puntos_right
        )

        puntos_3d = (
            puntos_4d[:3]
            / puntos_4d[3]
        ).T
        
        # Publicar nube
        cloud = PointCloud2()

        cloud.header = msg.header

        puntos = []

        for punto in puntos_3d:

            if not np.all(np.isfinite(punto)):
                continue

            puntos.append([
                float(punto[0]),
                float(punto[1]),
                float(punto[2])
            ])

        cloud = point_cloud2.create_cloud_xyz32(
            msg.header,
            puntos
        )

        return cloud

    def procesar_normal(self, msg: MatchingSet):
        cloud = self.procesar_matches(msg)
        
        if cloud is not None:
            self.pub.publish(cloud)
            
    def procesar_ransac(self, msg: MatchingSet):
        cloud = self.procesar_matches(msg)
        
        if cloud is not None:
            self.pub_ransac.publish(cloud)
    

def main(args=None):

    rclpy.init(args=args)

    node = Triangulation()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()