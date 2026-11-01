# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Bound total planar speed after smoothing and before collision monitoring."""

import math

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node


def limit_velocity(vx, vy, wz, max_speed, max_yaw_rate):
    """Preserve translation direction, cap speed, and reject nonfinite commands."""
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        return 0.0, 0.0, 0.0
    scale = min(1.0, max_speed / math.hypot(vx, vy)) if vx or vy else 1.0
    return vx * scale, vy * scale, max(-max_yaw_rate, min(max_yaw_rate, wz))


class PlanarVelocityLimiter(Node):
    """Apply the physical translation limit to every smoothed Twist command."""

    def __init__(self):
        """Configure limits, output, and a stale-command stop timer."""
        super().__init__('planar_velocity_limiter')
        fixed = ParameterDescriptor(read_only=True)
        self.max_speed = self.declare_parameter('max_speed', 0.63, fixed).value
        self.max_yaw_rate = self.declare_parameter('max_yaw_rate', 0.8, fixed).value
        self.timeout = self.declare_parameter('command_timeout', 0.5, fixed).value
        if not all(math.isfinite(v) and v > 0 for v in (
                self.max_speed, self.max_yaw_rate, self.timeout)):
            raise ValueError('Velocity limits and command_timeout must be finite and positive')
        self.last_command = None
        self.publisher = self.create_publisher(Twist, 'cmd_vel_limited', 1)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel_smoothed', self.on_command, 1)
        self.timer = self.create_timer(0.05, self.check_timeout)

    def on_command(self, command):
        """Publish a capped planar command, leaving collision monitoring last."""
        result = Twist()
        result.linear.x, result.linear.y, result.angular.z = limit_velocity(
            command.linear.x, command.linear.y, command.angular.z,
            self.max_speed, self.max_yaw_rate)
        self.last_command = self.get_clock().now()
        self.publisher.publish(result)

    def check_timeout(self):
        """Stop once when upstream commands go stale or simulation time resets."""
        if self.last_command is None:
            return
        age = (self.get_clock().now() - self.last_command).nanoseconds * 1e-9
        if age < 0 or age > self.timeout:
            self.publisher.publish(Twist())
            self.last_command = None


def main(args=None):
    """Run the planar velocity limiter."""
    rclpy.init(args=args)
    node = PlanarVelocityLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
