import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, PointStamped, Pose
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener, TransformException, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # needed for transform() with PointStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from dataclasses import dataclass
from typing import Optional
import numpy as np
from nav2_apps.helpers.robo_math import RoboMath


@dataclass
class Transform:
    parent_frame: str
    child_frame: str
    translation_x: float = 0.0
    translation_y: float = 0.0
    translation_z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def is_empty(self) -> bool:
        return (
            math.isnan(self.translation_x) and
            math.isnan(self.translation_y) and
            math.isnan(self.translation_z) and
            math.isnan(self.roll) and
            math.isnan(self.pitch) and
            math.isnan(self.yaw) and
            self.parent_frame == "" and
            self.child_frame == ""
        )


@dataclass
class Coordinates:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


class TfManager:
    def __init__(self, node: Node):
        self.node = node
        self.static_broadcaster = StaticTransformBroadcaster(node)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.robo_math_helper = RoboMath()

    def create_static_transform(self, new_transform: Transform) -> None:
        t = TransformStamped()

        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = new_transform.parent_frame
        t.child_frame_id = new_transform.child_frame

        t.transform.translation.x = new_transform.translation_x
        t.transform.translation.y = new_transform.translation_y
        t.transform.translation.z = new_transform.translation_z

        q = quaternion_from_euler(new_transform.roll, new_transform.pitch, new_transform.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        try:
            self.static_broadcaster.sendTransform(t)
            self.node.get_logger().info(
                f"New Transform published from {new_transform.parent_frame} to {new_transform.child_frame}"
            )
        except Exception as e:
            self.node.get_logger().error(
                f"Exception thrown while publishing new transform: {e}"
            )

    def get_tf_coords_parent_to_child(self, parent_frame: str, child_frame: str) -> Optional[Coordinates]:
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as e:
            self.node.get_logger().error(
                f"Could not find transform from {parent_frame} to {child_frame}.\nException: {e}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        roll, pitch, yaw = euler_from_quaternion([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        ])

        return Coordinates(
            x=translation.x,
            y=translation.y,
            z=translation.z,
            roll=roll,
            pitch=pitch,
            yaw=yaw
        )

    def move_subject_towards_target(self, subject: Coordinates, target: Coordinates) -> Twist:
        # --- Position error ---
        dx = target.x - subject.x
        dy = target.y - subject.y
        distance = math.sqrt(dx**2 + dy**2)

        # --- Angle to target position ---
        dyaw = math.atan2(dy, dx)

        # --- Heading error, normalized to [-pi, pi] ---
        yaw_error = dyaw - subject.yaw
        while yaw_error >  math.pi: yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi: yaw_error += 2.0 * math.pi

        kp_yaw = 1.0
        if -0.1 < yaw_error < 0.1:
            kp_distance = 0.25
        else:
            kp_distance = 0.10

        cmd = Twist()
        cmd.linear.x = kp_distance * distance
        cmd.angular.z = kp_yaw * yaw_error

        return cmd

    def check_if_tf_exists(self, parent_frame: str, child_frame: str) -> bool:
        return self.tf_buffer.can_transform(
            child_frame,
            parent_frame,
            rclpy.time.Time()
        )

    def transform_point(self, point: PointStamped, target_frame: str) -> Optional[PointStamped]:
        try:
            return self.tf_buffer.transform(
                point,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as ex:
            self.node.get_logger().warn(f"Could not transform point: {ex}")
            return None

    def get_frame_yaw_in_parent(self, child_frame, parent_frame):
        """
        Returns the yaw (rotation about Z) of child_frame expressed in parent_frame,
        or None if the transform lookup fails.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),  # latest available transform
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException,
                ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(
                f"Could not get transform from {child_frame} to {parent_frame}: {e}"
            )
            return None

        q = transform.transform.rotation
        yaw = self.robo_math_helper.quaternion_to_yaw(q)
        return yaw