# Copyright (c) 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bridge Nav2 Twist commands to the swerve controller's TwistStamped input."""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


class TwistToTwistStampedBridge(Node):
    """Republish unstamped Twist commands as TwistStamped commands."""

    def __init__(self):
        super().__init__('twist_to_twist_stamped_bridge')

        self.declare_parameter('input_topic', 'cmd_vel_nav')
        self.declare_parameter('output_topic', 'swerve_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.subscription = self.create_subscription(Twist, self.input_topic, self._callback, 10)

        self.get_logger().info(
            'Twist to TwistStamped bridge ready: '
            f'{self.input_topic} -> {self.output_topic} ({self.frame_id})'
        )

    def _callback(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStampedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
