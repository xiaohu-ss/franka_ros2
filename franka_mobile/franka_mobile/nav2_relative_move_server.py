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

"""Relative move service for the TMR base.

Short moves run a closed-loop proportional controller straight against the
odometry TF, driving x, y and yaw to zero simultaneously.  Longer moves are
handed to Nav2's NavigateToPose, which brings obstacle avoidance along.

The split exists because Nav2 cannot service a decimetre-scale pose request:
NavFn plans on a 5 cm grid, DWB scores sampled trajectories rather than
servoing on the error, and its RotateToGoal critic latches into a
rotate-in-place phase that rejects any further translation.  None of that
converges to a centimetre.
"""

from math import atan2, cos, hypot, pi, sin
from threading import Event, Lock
import time

from action_msgs.msg import GoalStatus
from franka_msgs.srv import Nav2RelativeMove
from geometry_msgs.msg import TwistStamped
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
DEFAULT_RESULT_TIMEOUT = 60.0
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


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle into [-pi, pi)."""
    return (angle + pi) % (2.0 * pi) - pi


def clamp(value: float, limit: float) -> float:
    """Clamp a value to +/- limit."""
    return max(-limit, min(limit, value))


def apply_floor(value: float, floor: float) -> float:
    """Raise a non-zero command to at least the floor magnitude."""
    if value == 0.0 or floor <= 0.0 or abs(value) >= floor:
        return value
    return floor if value > 0.0 else -floor


def goal_status_to_string(status: int) -> str:
    """Return a human-readable action goal status."""
    status_names = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return status_names.get(status, f'UNKNOWN_STATUS_{status}')


class Nav2RelativeMoveServer(Node):
    """Converts relative base-frame motion requests into base motion."""

    def __init__(self):
        super().__init__('nav2_relative_move_server')

        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('action_wait_timeout', DEFAULT_ACTION_WAIT_TIMEOUT)
        self.declare_parameter('goal_response_timeout', DEFAULT_GOAL_RESPONSE_TIMEOUT)
        self.declare_parameter('result_timeout', DEFAULT_RESULT_TIMEOUT)
        self.declare_parameter('tf_lookup_timeout', DEFAULT_TF_LOOKUP_TIMEOUT)

        # Closed-loop servo
        self.declare_parameter('cmd_vel_topic', 'swerve_drive_controller/cmd_vel')
        self.declare_parameter('cmd_vel_frame', 'base_link')
        self.declare_parameter('closed_loop_max_distance', 0.5)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('kp_xy', 1.5)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('max_vel_xy', 0.10)
        self.declare_parameter('max_vel_yaw', 0.4)
        self.declare_parameter('min_vel_xy', 0.015)
        self.declare_parameter('min_vel_yaw', 0.03)
        self.declare_parameter('max_accel_xy', 0.3)
        self.declare_parameter('max_accel_yaw', 0.3)
        self.declare_parameter('xy_tolerance', 0.01)
        self.declare_parameter('yaw_tolerance', 0.02)
        self.declare_parameter('settle_time', 0.3)
        self.declare_parameter('closed_loop_timeout', 30.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        navigate_to_pose_action = self.get_parameter('navigate_to_pose_action').value
        self.action_wait_timeout = self.get_parameter('action_wait_timeout').value
        self.goal_response_timeout = self.get_parameter('goal_response_timeout').value
        self.result_timeout = self.get_parameter('result_timeout').value
        self.tf_lookup_timeout = self.get_parameter('tf_lookup_timeout').value

        self.cmd_vel_frame = self.get_parameter('cmd_vel_frame').value
        self.closed_loop_max_distance = self.get_parameter('closed_loop_max_distance').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.kp_xy = self.get_parameter('kp_xy').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_vel_xy = self.get_parameter('max_vel_xy').value
        self.max_vel_yaw = self.get_parameter('max_vel_yaw').value
        self.min_vel_xy = self.get_parameter('min_vel_xy').value
        self.min_vel_yaw = self.get_parameter('min_vel_yaw').value
        self.max_accel_xy = self.get_parameter('max_accel_xy').value
        self.max_accel_yaw = self.get_parameter('max_accel_yaw').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.settle_time = self.get_parameter('settle_time').value
        self.closed_loop_timeout = self.get_parameter('closed_loop_timeout').value

        self.callback_group = ReentrantCallbackGroup()
        # spin_thread keeps TF filling while a service callback blocks on the
        # control loop.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
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
        self.move_lock = Lock()

        self.get_logger().info(
            'Relative move service ready: '
            f'{self.robot_base_frame} relative moves in {self.target_frame}, '
            f'closed loop below {self.closed_loop_max_distance:.2f} m, Nav2 above'
        )

    # ------------------------------------------------------------------ utils

    def _lookup_pose(self):
        """Return the base pose in the target frame as (x, y, yaw)."""
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.robot_base_frame,
            Time(),
            timeout=Duration(seconds=self.tf_lookup_timeout),
        )
        translation = transform.transform.translation
        return translation.x, translation.y, yaw_from_quaternion(transform.transform.rotation)

    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        """Publish one body-frame velocity command."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.cmd_vel_frame
        message.twist.linear.x = vx
        message.twist.linear.y = vy
        message.twist.angular.z = wz
        self.cmd_vel_publisher.publish(message)

    def _stop(self) -> None:
        """Command a full stop, repeated so a dropped message cannot run away."""
        for _ in range(3):
            self._publish_cmd(0.0, 0.0, 0.0)
            time.sleep(0.02)

    # ------------------------------------------------------------- closed loop

    def _run_closed_loop(self, target_x, target_y, target_yaw, response):
        """Servo the base onto the target pose, all three axes at once."""
        period = 1.0 / self.control_frequency
        deadline = time.monotonic() + self.closed_loop_timeout
        in_tolerance_since = None
        vx_previous = 0.0
        vy_previous = 0.0
        wz_previous = 0.0

        while True:
            if not rclpy.ok():
                self._stop()
                response.accepted = False
                response.message = 'Closed-loop move aborted: shutting down'
                return response

            if time.monotonic() > deadline:
                self._stop()
                error_xy, error_yaw = self._pose_error(target_x, target_y, target_yaw)
                response.accepted = False
                response.message = (
                    'Closed-loop move timed out: '
                    f'timeout={self.closed_loop_timeout:.1f}s, '
                    f'remaining_xy={error_xy:.3f}m, remaining_yaw={error_yaw:.3f}rad'
                )
                return response

            try:
                current_x, current_y, current_yaw = self._lookup_pose()
            except TransformException as exc:
                self._stop()
                response.accepted = False
                response.message = (
                    'Closed-loop move aborted, lost '
                    f'{self.target_frame} -> {self.robot_base_frame}: {exc}'
                )
                return response

            # Error in the target frame, rotated into the body frame so the
            # command axes line up with the base.
            error_x_world = target_x - current_x
            error_y_world = target_y - current_y
            error_x = cos(current_yaw) * error_x_world + sin(current_yaw) * error_y_world
            error_y = -sin(current_yaw) * error_x_world + cos(current_yaw) * error_y_world
            error_yaw = wrap_to_pi(target_yaw - current_yaw)
            distance = hypot(error_x, error_y)

            if distance <= self.xy_tolerance and abs(error_yaw) <= self.yaw_tolerance:
                if in_tolerance_since is None:
                    in_tolerance_since = time.monotonic()
                elif time.monotonic() - in_tolerance_since >= self.settle_time:
                    self._stop()
                    response.accepted = True
                    response.message = (
                        'Closed-loop move finished: '
                        f'x={current_x:.3f}, y={current_y:.3f}, yaw={current_yaw:.3f}, '
                        f'error_xy={distance:.4f}, error_yaw={error_yaw:.4f}'
                    )
                    return response
            else:
                in_tolerance_since = None

            vx = self.kp_xy * error_x
            vy = self.kp_xy * error_y
            wz = clamp(self.kp_yaw * error_yaw, self.max_vel_yaw)

            # Scale the translation as a vector so the path stays a straight
            # line; clamping the axes one by one would bend it.
            speed = hypot(vx, vy)
            if speed > self.max_vel_xy:
                scale = self.max_vel_xy / speed
                vx *= scale
                vy *= scale
                speed = self.max_vel_xy

            if distance > self.xy_tolerance and 0.0 < speed < self.min_vel_xy:
                scale = self.min_vel_xy / speed
                vx *= scale
                vy *= scale
            if abs(error_yaw) > self.yaw_tolerance:
                wz = apply_floor(wz, self.min_vel_yaw)

            if distance <= self.xy_tolerance:
                vx = 0.0
                vy = 0.0
            if abs(error_yaw) <= self.yaw_tolerance:
                wz = 0.0

            # Slew limit so the swerve modules are never asked for a step
            # change in direction.
            step_xy = self.max_accel_xy * period
            step_yaw = self.max_accel_yaw * period
            vx = vx_previous + clamp(vx - vx_previous, step_xy)
            vy = vy_previous + clamp(vy - vy_previous, step_xy)
            wz = wz_previous + clamp(wz - wz_previous, step_yaw)
            vx_previous, vy_previous, wz_previous = vx, vy, wz

            self._publish_cmd(vx, vy, wz)
            time.sleep(period)

    def _pose_error(self, target_x, target_y, target_yaw):
        """Return (distance, yaw error) to the target, or (nan, nan) on TF failure."""
        try:
            current_x, current_y, current_yaw = self._lookup_pose()
        except TransformException:
            return float('nan'), float('nan')
        return (
            hypot(target_x - current_x, target_y - current_y),
            wrap_to_pi(target_yaw - current_yaw),
        )

    # -------------------------------------------------------------------- nav2

    def _run_nav2(self, target_x, target_y, target_yaw, response):
        """Hand the target pose to Nav2 and report where the base ended up."""
        if not self.navigate_to_pose_client.wait_for_server(
            timeout_sec=self.action_wait_timeout
        ):
            response.accepted = False
            response.message = 'NavigateToPose action server is not available'
            return response

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

        if not goal_handle.accepted:
            response.accepted = False
            response.message = 'NavigateToPose goal rejected'
            return response

        result_future = goal_handle.get_result_async()
        result_event = Event()
        result_future.add_done_callback(lambda _future: result_event.set())

        if not result_event.wait(timeout=self.result_timeout):
            response.accepted = False
            response.message = (
                'Timed out waiting for NavigateToPose result: '
                f'timeout={self.result_timeout:.3f}s'
            )
            return response

        status = result_future.result().status
        status_name = goal_status_to_string(status)

        try:
            final_x, final_y, final_yaw = self._lookup_pose()
        except TransformException as exc:
            response.accepted = False
            response.message = (
                'NavigateToPose finished, but failed to lookup final pose: '
                f'status={status_name}, '
                f'{self.target_frame} -> {self.robot_base_frame}: {exc}'
            )
            return response

        response.accepted = status == GoalStatus.STATUS_SUCCEEDED
        response.message = (
            'NavigateToPose finished: '
            f'status={status_name}, '
            f'x={final_x:.3f}, y={final_y:.3f}, yaw={final_yaw:.3f}'
        )
        return response

    # ----------------------------------------------------------------- service

    def _relative_move_callback(self, request, response):
        if not self.move_lock.acquire(blocking=False):
            response.accepted = False
            response.message = 'A relative move is already in progress'
            return response

        try:
            try:
                current_x, current_y, current_yaw = self._lookup_pose()
            except TransformException as exc:
                response.accepted = False
                response.message = (
                    f'Failed to lookup {self.target_frame} -> {self.robot_base_frame}: {exc}'
                )
                return response

            target_x = (
                current_x
                + cos(current_yaw) * request.forward_m
                - sin(current_yaw) * request.left_m
            )
            target_y = (
                current_y
                + sin(current_yaw) * request.forward_m
                + cos(current_yaw) * request.left_m
            )
            target_yaw = wrap_to_pi(current_yaw + request.yaw_rad)

            distance = hypot(request.forward_m, request.left_m)
            if distance <= self.closed_loop_max_distance:
                self.get_logger().info(
                    f'Closed-loop relative move: forward={request.forward_m:.3f}m, '
                    f'left={request.left_m:.3f}m, yaw={request.yaw_rad:.3f}rad'
                )
                return self._run_closed_loop(target_x, target_y, target_yaw, response)

            self.get_logger().info(
                f'Nav2 relative move ({distance:.2f}m > '
                f'{self.closed_loop_max_distance:.2f}m): '
                f'forward={request.forward_m:.3f}m, left={request.left_m:.3f}m, '
                f'yaw={request.yaw_rad:.3f}rad'
            )
            return self._run_nav2(target_x, target_y, target_yaw, response)
        finally:
            self.move_lock.release()


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
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
