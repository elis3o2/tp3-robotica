import csv

import numpy as np
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from transforms3d.quaternions import quat2mat

from ament_index_python.packages import get_package_share_directory
import os


class Mapping(Node):

    def __init__(self):
        super().__init__('mapping')

        package_share = get_package_share_directory('tp3')

        ruta_csv = os.path.join(
            package_share,
            'groundtruth_camera_left.csv'
        )

        self.timestamps_gt = None
        self.posiciones_gt = None
        self.rotaciones_gt = None

        self.cargar_groundtruth(ruta_csv)

        self.puntos_mapa = []
        self.timestamp_anterior = None

        self.sub_points = self.create_subscription(
            PointCloud2, '/ransac_points3d', self.procesar_nube, 10
        )

        self.pub_map = self.create_publisher(
            PointCloud2, '/map_points3d', 10
        )

        self.get_logger().info(
            'Nodo de mapping iniciado'
        )


    def cargar_groundtruth(self, ruta):

        timestamps = []
        posiciones = []
        rotaciones = []

        with open(ruta, 'r') as archivo:

            reader = csv.DictReader(archivo)

            for fila in reader:

                timestamp = int(fila['timestamp'])
                
                posicion = np.array(
                    [
                        float(fila['x']),
                        float(fila['y']),
                        float(fila['z'])
                    ],
                    dtype=np.float64
                )

                qw = float(fila['qw'])
                qx = float(fila['qx'])
                qy = float(fila['qy'])
                qz = float(fila['qz'])

                R = quat2mat([qw, qx, qy, qz])

                timestamps.append(timestamp)

                posiciones.append(posicion)

                rotaciones.append(R)

        self.timestamps_gt = np.array(timestamps, dtype=np.int64)

        self.posiciones_gt = np.array(posiciones, dtype=np.float64)

        self.rotaciones_gt = np.array(rotaciones, dtype=np.float64)


    def timestamp_ns(self, stamp):

        return stamp.sec * 1_000_000_000 + stamp.nanosec


    def buscar_pose(self, timestamp):

        indice = np.searchsorted(self.timestamps_gt, timestamp)

        if indice == 0:

            indice_mejor = 0

        elif indice >= len(self.timestamps_gt):

            indice_mejor = len(self.timestamps_gt) - 1

        else:

            anterior = indice - 1
            siguiente = indice

            diferencia_anterior = abs(int(self.timestamps_gt[anterior]) - timestamp)

            diferencia_siguiente = abs(int(self.timestamps_gt[siguiente]) - timestamp)

            if diferencia_anterior <= diferencia_siguiente:
                indice_mejor = anterior
            
            else:
                indice_mejor = siguiente

        posicion = self.posiciones_gt[indice_mejor]

        rotacion = self.rotaciones_gt[indice_mejor]

        timestamp_gt = int(self.timestamps_gt[indice_mejor])

        diferencia_ms = abs(timestamp_gt - timestamp) / 1e6

        return (
            posicion,
            rotacion,
            diferencia_ms
        )


    def procesar_nube(self, msg: PointCloud2):

        if msg.width == 0:
            return

        timestamp = self.timestamp_ns(msg.header.stamp)

        if self.timestamp_anterior is not None and timestamp < self.timestamp_anterior:
            self.puntos_mapa = []

        self.timestamp_anterior = timestamp

        posicion, rotacion, diferencia_ms = self.buscar_pose(timestamp)

        puntos = point_cloud2.read_points(
            msg,
            field_names=(
                'x',
                'y',
                'z',
                'rgb'
            ),
            skip_nans=True
        )

        nuevos_puntos = []

        for punto in puntos:

            x = float(punto[0])
            y = float(punto[1])
            z = float(punto[2])
            rgb = float(punto[3])

            punto_camara = np.array(
                [x, y, z],
                dtype=np.float64
            )

            punto_mundo = rotacion @ punto_camara + posicion

            nuevos_puntos.append(
                [
                    float(punto_mundo[0]),
                    float(punto_mundo[1]),
                    float(punto_mundo[2]),
                    rgb
                ]
            )

        self.puntos_mapa.extend(nuevos_puntos)


        self.publicar_mapa(
            msg.header.stamp
        )


    def publicar_mapa(self, stamp):

        if len(self.puntos_mapa) == 0:
            return

        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),

            PointField(
                name='rgb',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        header = Header()

        header.stamp = stamp

        header.frame_id = 'map'

        cloud = point_cloud2.create_cloud(
            header,
            fields,
            self.puntos_mapa
        )
        

        self.get_logger().info(
            f'Total puntos mapa: {len(self.puntos_mapa)}'
        )

        self.get_logger().info(
            f'Primeros puntos: {self.puntos_mapa[:10]}'
        )  
         
        self.pub_map.publish(
            cloud
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = Mapping()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()