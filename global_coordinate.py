import math
import time
import curses

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class GlobalCoordinate(Node):

    def __init__(self):
        super().__init__('global_coordinate')

        self.r = 0.05
        self.L = 0.50

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        self.wl = 0.0
        self.wr = 0.0

        self.last_time = time.monotonic()

        self.pub = self.create_publisher(
            Twist,
            '/model/vehicle_blue/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.update_position
        )

    def update_position(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0:
            return

        v = self.r * (self.wr + self.wl) / 2.0
        omega = self.r * (self.wr - self.wl) / self.L

        self.vx = v * math.cos(self.theta)
        self.vy = v * math.sin(self.theta)
        self.vtheta = omega

        theta_mid = self.theta + omega * dt / 2.0

        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += omega * dt

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega

        self.pub.publish(cmd)


def terminal_ui(stdscr, node):
    curses.curs_set(1)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    input_buffer = ''
    running = True
    message = 'Masukkan kecepatan roda kiri dan kanan.'

    while running and rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.01)

        key = stdscr.getch()

        if key != -1:

            if key in (ord('q'), ord('Q')):
                running = False

            elif key in (10, 13, curses.KEY_ENTER):
                try:
                    values = input_buffer.strip().split()

                    if len(values) != 2:
                        raise ValueError

                    wl = float(values[0])
                    wr = float(values[1])

                    node.wl = wl
                    node.wr = wr

                    message = (
                        f'Kecepatan diterima: '
                        f'kiri={wl:.2f} rad/s, '
                        f'kanan={wr:.2f} rad/s'
                    )

                except ValueError:
                    message = (
                        'Input salah! Masukkan dua angka, '
                        'contoh: 4 4'
                    )

                input_buffer = ''

            elif key in (curses.KEY_BACKSPACE, 127, 8):
                input_buffer = input_buffer[:-1]

            elif 32 <= key <= 126:
                input_buffer += chr(key)

        x = node.x
        y = node.y
        theta = node.theta

        vx = node.vx
        vy = node.vy
        vtheta = node.vtheta

        stdscr.erase()

        stdscr.addstr(0, 2, '=== GLOBAL COORDINATE ROBOT ===')
        stdscr.addstr(2, 2, 'POSISI GLOBAL')
        stdscr.addstr(3, 2, f'X     : {x:10.4f} m')
        stdscr.addstr(4, 2, f'Y     : {y:10.4f} m')
        stdscr.addstr(5, 2, f'Theta : {theta:10.4f} rad')

        stdscr.addstr(7, 2, 'KECEPATAN GLOBAL')
        stdscr.addstr(8, 2, f'Vx     : {vx:10.4f} m/s')
        stdscr.addstr(9, 2, f'Vy     : {vy:10.4f} m/s')
        stdscr.addstr(10, 2, f'Vtheta : {vtheta:10.4f} rad/s')

        stdscr.addstr(12, 2, 'KECEPATAN RODA')
        stdscr.addstr(13, 2, f'Roda kiri  : {node.wl:10.2f} rad/s')
        stdscr.addstr(14, 2, f'Roda kanan : {node.wr:10.2f} rad/s')

        stdscr.addstr(16, 2, 'INPUT KECEPATAN')
        stdscr.addstr(
            17, 2,
            'Ketik: kecepatan_kiri kecepatan_kanan'
        )
        stdscr.addstr(
            18, 2,
            f'Input: {input_buffer}'
        )

        stdscr.addstr(20, 2, message)
        stdscr.addstr(22, 2, 'Contoh: 4 4 = maju | 0 0 = berhenti')
        stdscr.addstr(23, 2, 'Tekan q untuk keluar.')

        stdscr.refresh()

    node.wl = 0.0
    node.wr = 0.0

    stop_cmd = Twist()
    node.pub.publish(stop_cmd)

    rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCoordinate()

    try:
        curses.wrapper(terminal_ui, node)

    except KeyboardInterrupt:
        pass

    finally:
        node.wl = 0.0
        node.wr = 0.0

        stop_cmd = Twist()
        node.pub.publish(stop_cmd)

        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.shutdown()

        node.destroy_node()
        print('Program Global Coordinate selesai.')


if __name__ == '__main__':
    main()