#!/usr/bin/python3
"""
Keyboard GPS fault injection for the FCU HITL simulation.

  g  - NO GPS on/off     (no GPS messages are sent to the FCU)
  G  - CORRUPTED on/off  (GPS messages are sent, but flagged invalid: no fix, 3 satellites, DataValid = 0)
  s  - print the current state
  q  - quit (Ctrl-C works as well)

The two faults are mutually exclusive, pressing g while CORRUPTED is active switches directly to NO GPS and vice versa.
The keys only work while this terminal pane has focus.

Usage: ./gps_toggle.py [service_prefix]   (default /$UAV_NAME/hw_api/hitl)
"""

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool

MODE_NORMAL = 0
MODE_NO_GPS = 1
MODE_CORRUPTED = 2
MODE_NAMES = {MODE_NORMAL: 'NORMAL', MODE_NO_GPS: 'NO GPS', MODE_CORRUPTED: 'CORRUPTED'}


class GpsToggle(Node):

    def __init__(self, prefix):

        use_sim_time = os.environ.get('USE_SIM_TIME', 'false').lower() == 'true'
        super().__init__('gps_toggle', parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])

        self.mode = None  # unknown until the latched mode topic arrives

        # The mode topic is latched (transient local) in hw_api, a restarted script learns the current mode immediately
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UInt8, f'{prefix}/gps_fault_mode', self.callback_mode, qos)

        self.cli_no_gps = self.create_client(SetBool, f'{prefix}/gps_no_gps')
        self.cli_corrupted = self.create_client(SetBool, f'{prefix}/gps_corrupted')

        self.prefix = prefix

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def print_state(self):
        name = MODE_NAMES.get(self.mode, 'UNKNOWN (waiting for hw_api)')
        print(f'[t = {self.now_s():9.2f} s]  GPS: {name}', flush=True)

    def callback_mode(self, msg):
        if msg.data != self.mode:
            self.mode = msg.data
            self.print_state()

    def toggle(self, client, fault_mode):
        if not client.service_is_ready():
            print(f'Service {client.srv_name} is not available - is hw_api running?', flush=True)
            return

        request = SetBool.Request()
        request.data = (self.mode != fault_mode)  # fault active -> switch it off, otherwise switch it on
        future = client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
        except Exception as e:  # noqa: BLE001 - report any service failure to the user
            print(f'Service call failed: {e}', flush=True)
            return
        if not response.success:
            print(f'Service refused the request: {response.message}', flush=True)
        # the new state is printed by callback_mode once hw_api publishes it


def main():

    if not sys.stdin.isatty():
        print('gps_toggle.py needs an interactive terminal', file=sys.stderr)
        return 1

    uav_name = os.environ.get('UAV_NAME', 'uav1')
    prefix = sys.argv[1] if len(sys.argv) > 1 else f'/{uav_name}/hw_api/hitl'

    rclpy.init()
    node = GpsToggle(prefix)

    print(__doc__.split('Usage')[0].strip(), flush=True)
    print(f'\nServices: {prefix}/gps_no_gps, {prefix}/gps_corrupted\n', flush=True)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)

    try:
        # cbreak - single key presses without Enter, Ctrl-C and normal printing keep working
        tty.setcbreak(fd)

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                continue

            key = sys.stdin.read(1)

            if key == 'g':
                node.toggle(node.cli_no_gps, MODE_NO_GPS)
            elif key == 'G':
                node.toggle(node.cli_corrupted, MODE_CORRUPTED)
            elif key == 's':
                node.print_state()
            elif key == 'q':
                break

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
