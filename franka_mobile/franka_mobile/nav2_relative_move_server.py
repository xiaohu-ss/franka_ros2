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

"""Relative move service backed by Nav2 NavigateToPose."""

from math import atan2, cos, sin
from threading import Event

from franka_msgs.srv import Nav2RelativeMove
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

DEFAULT_ACTION_WAIT_TIMEOUT = 5.0
DEFAULT_GOAL_RESPONSE_TIMEOUT = 5.0
DEFAULT_TF_LOOKUP_TIMEOUT = 1.0


def yaw_from_quaternion(quaternion) -> float:
    """Return yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return atan2(siny_cosp, cosy_cosp)


def set_quaternion_from_yaw(quaternion, yaw: float) -> None:
    """Set a geometry_msgs Quaternion to a planar yaw rotation."""
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(yaw * 0.5)
    quaternion.w = cos(yaw * 0.5)


class Nav2RelativeMoveServer(Node):
    """Converts relative base-frame motion requests into Nav2 NavigateToPose goals."""

    def __init__(self):
        super().__init__('nav2_relative_move_server')

        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('action_wait_timeout', DEFAULT_ACTION_WAIT_TIMEOUT)
        self.declare_parameter('goal_response_timeout', DEFAULT_GOAL_RESPONSE_TIMEOUT)
        self.declare_parameter('tf_lookup_timeout', DEFAULT_TF_LOOKUP_TIMEOUT)

        self.target_frame = self.get_parameter('target_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        navigate_to_pose_action = self.get_parameter('navigate_to_pose_action').value
        self.action_wait_timeout = self.get_parameter('action_wait_timeout').value
        self.goal_response_timeout = self.get_parameter('goal_response_timeout').value
        self.tf_lookup_timeout = self.get_parameter('tf_lookup_timeout').value

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            navigate_to_pose_action,
            callback_group=self.callback_group,
        )
        self.relative_move_service = self.create_service(
            Nav2RelativeMove,
            '~/relative_move',
            self._relative_move_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Nav2 relative move service ready: '
            f'{self.robot_base_frame} relative moves in {self.target_frame}'
        )

    def _relative_move_callback(self, request, response):
        if not self.navigate_to_pose_client.wait_for_server(
            timeout_sec=self.action_wait_timeout
        ):
            response.accepted = False
            response.message = 'NavigateToPose action server is not available'
            return response

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
        except TransformException as exc:
            response.accepted = False
            response.message = (
                f'Failed to lookup {self.target_frame} -> {self.robot_base_frame}: {exc}'
            )
            return response

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        current_yaw = yaw_from_quaternion(rotation)

        target_x = (
            translation.x
            + cos(current_yaw) * request.forward_m
            - sin(current_yaw) * request.left_m
        )
        target_y = (
            translation.y
            + sin(current_yaw) * request.forward_m
            + cos(current_yaw) * request.left_m
        )
        target_yaw = current_yaw + request.yaw_rad

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.target_frame
        goal.pose.pose.position.x = target_x
        goal.pose.pose.position.y = target_y
        goal.pose.pose.position.z = 0.0
        set_quaternion_from_yaw(goal.pose.pose.orientation, target_yaw)

        goal_future = self.navigate_to_pose_client.send_goal_async(goal)
        done_event = Event()
        goal_future.add_done_callback(lambda _future: done_event.set())

        if not done_event.wait(timeout=self.goal_response_timeout):
            response.accepted = False
            response.message = 'Timed out waiting for NavigateToPose goal response'
            return response

        goal_handle = goal_future.result()
        if goal_handle is None:
            response.accepted = False
            response.message = 'NavigateToPose goal response was empty'
            return response

        response.accepted = goal_handle.accepted
        if goal_handle.accepted:
            response.message = (
                'NavigateToPose goal accepted: '
                f'x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw:.3f}'
            )
        else:
            response.message = 'NavigateToPose goal rejected'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Nav2RelativeMoveServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
