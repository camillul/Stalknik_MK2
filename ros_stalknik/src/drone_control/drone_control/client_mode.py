import sys

import rclpy
from rclpy.node import Node
from drone_interfaces.srv import FollowUp


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('client_mode')
        self.cli = self.create_client(FollowUp, 'follow_up_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FollowUp.Request()

    def send_request(self):
        self.req.mode = int(sys.argv[1])
        self.future = self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Follow up mode changed' )
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()