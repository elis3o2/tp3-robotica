import sys
import tty
import termios
import select

import rclpy

from rclpy.node import Node

from rosbag2_interfaces.srv import TogglePaused, PlayNext, GetRate, SetRate


class BagControls(Node):

    def __init__(self):
        super().__init__('bag_controls')

        self.toggle_client = self.create_client(
            TogglePaused, '/rosbag2_player/toggle_paused'
        )

        self.next_client = self.create_client(
            PlayNext, '/rosbag2_player/play_next'
        )

        self.get_rate_client = self.create_client(
            GetRate, '/rosbag2_player/get_rate'
        )

        self.set_rate_client = self.create_client(
            SetRate, '/rosbag2_player/set_rate'
        )

        self.rate = 1.0

        self.esperar_servicios()
        self.obtener_rate()

        print()
        print('Controles ROS Bag')
        print('-----------------')
        print('SPACE : Pausa / Reanuda')
        print('↑     : Aumentar velocidad 10%')
        print('↓     : Disminuir velocidad 10%')
        print('→     : Siguiente mensaje')
        print('q     : Salir')
        print()


    def esperar_servicios(self):

        clientes = [
            self.toggle_client,
            self.next_client,
            self.get_rate_client,
            self.set_rate_client
        ]

        for cliente in clientes:
            while not cliente.wait_for_service(
                timeout_sec=1.0
            ):
                self.get_logger().info(
                    'Esperando rosbag2_player...'
                )


    def obtener_rate(self):

        request = GetRate.Request()

        future = self.get_rate_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self, future
        )

        if future.result() is not None:
            self.rate = future.result().rate

        print(f'Velocidad actual: {self.rate:.2f}x')


    def toggle_pause(self):

        request = TogglePaused.Request()

        self.toggle_client.call_async(
            request
        )

        print('Pausa / Reanudar')


    def siguiente(self):

        request = PlayNext.Request()

        self.next_client.call_async(
            request
        )

        print('Siguiente mensaje')


    def cambiar_rate(self, nuevo_rate):

        if nuevo_rate <= 0:
            return

        request = SetRate.Request()

        request.rate = float(
            nuevo_rate
        )

        future = self.set_rate_client.call_async(
            request
        )

        self.rate = nuevo_rate

        print(
            f'Velocidad: {self.rate:.2f}x'
        )


    def aumentar_rate(self):

        self.cambiar_rate(
            self.rate * 1.1
        )


    def disminuir_rate(self):

        self.cambiar_rate(
            self.rate / 1.1
        )


def main(args=None):

    rclpy.init(args=args)

    node = BagControls()

    # Guardar configuración original de terminal
    configuracion_terminal = termios.tcgetattr(
        sys.stdin
    )

    try:

        # Permite leer teclas sin presionar Enter
        tty.setcbreak(
            sys.stdin.fileno()
        )

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

            disponible, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0.05
            )

            if not disponible:
                continue

            tecla = sys.stdin.read(1)

            # SPACE
            if tecla == ' ':
                node.toggle_pause()

            # q
            elif tecla == 'q':
                break

            # Flechas
            elif tecla == '\x1b':

                segunda = sys.stdin.read(1)

                if segunda != '[':
                    continue

                tercera = sys.stdin.read(1)

                # ↑
                if tercera == 'A':
                    node.aumentar_rate()

                # ↓
                elif tercera == 'B':
                    node.disminuir_rate()

                # →
                elif tercera == 'C':
                    node.siguiente()

    except KeyboardInterrupt:
        pass

    finally:

        # Recuperar terminal normal
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            configuracion_terminal
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()