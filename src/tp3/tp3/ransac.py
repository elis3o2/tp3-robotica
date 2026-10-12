import cv2
import rclpy
import numpy as np

from rclpy.node import Node

from tp3_interfaces.msg import MatchingSet, MatrizTimeSet


class Ransac(Node):

    def __init__(self):
        super().__init__('ransac')

        # Recibo matches originales
        self.sub_matching = self.create_subscription(
            MatchingSet, '/matchings', self.procesar_matches, 10
        )

        # Publico solamente los matches aceptados por RANSAC
        self.pub_matchings = self.create_publisher(
            MatchingSet, '/ransac_matchings', 10
        )

        # Publico la matriz fundamental
        self.pub_matriz = self.create_publisher(
            MatrizTimeSet, '/f_matriz', 10
        )

        self.get_logger().info('RANSAC iniciado')


    def puntos_a_array(self, puntos):

        return np.array(
            [[p.x, p.y] for p in puntos],
            dtype=np.float32
        )


    def procesar_matches(self, msg: MatchingSet):

        # Para calcular F necesitamos al menos 8 correspondencias
        if len(msg.keypoints_left) < 8:
            self.get_logger().info(
                'No hay suficientes matches para calcular F'
            )
            return

        if len(msg.keypoints_left) != len(msg.keypoints_right):
            self.get_logger().error(
                'La cantidad de puntos izquierdos y derechos no coincide'
            )
            return

        # Puntos a array 
        puntos_left = self.puntos_a_array(msg.keypoints_left)

        puntos_right = self.puntos_a_array(msg.keypoints_right)

        # Aplico RANSAC 
        F, mascara = cv2.findFundamentalMat(
            puntos_left,
            puntos_right,
            cv2.FM_RANSAC,
        )

        if F is None or mascara is None:
            self.get_logger().info(
                'No se pudo calcular la matriz fundamental'
            )
            return

        # La máscara pasa de 0/1 a False/True
        mascara = mascara.ravel().astype(bool)

        # Publicar matches aceptados
        matches_filtrados = MatchingSet()

        matches_filtrados.header = msg.header

        indices_inliers = np.where(mascara)[0]

        for i in indices_inliers:
            matches_filtrados.keypoints_left.append(msg.keypoints_left[i])
            matches_filtrados.keypoints_right.append(msg.keypoints_right[i])
            matches_filtrados.distances.append(msg.distances[i])

        self.pub_matchings.publish(matches_filtrados)

        # Publicar matriz fundamental
        matriz_msg = MatrizTimeSet()

        matriz_msg.header = msg.header

        matriz_msg.matriz_f = (F.flatten().tolist())

        self.pub_matriz.publish(matriz_msg)




def main(args=None):

    rclpy.init(args=args)

    node = Ransac()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()