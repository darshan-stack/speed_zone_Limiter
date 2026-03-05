#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from shapely.geometry import Point, Polygon


class SpeedZoneLimiter(Node):
    def __init__(self):
        super().__init__('speed_zone_limiter')

        # Declare parameters with defaults
        self.declare_parameter('zone.name', 'unnamed_zone')
        self.declare_parameter('zone.max_speed', 0.1)
        self.declare_parameter('zone.polygon', [1.2, 0.5, 3.4, 0.5, 3.4, 2.1, 1.2, 2.1])

        # Load zone config
        self.zone_name = self.get_parameter('zone.name').get_parameter_value().string_value
        self.max_speed = self.get_parameter('zone.max_speed').get_parameter_value().double_value
        flat_polygon = self.get_parameter('zone.polygon').get_parameter_value().double_array_value

        # Build shapely polygon from flat [x0, y0, x1, y1, ...] list
        coords = list(zip(flat_polygon[0::2], flat_polygon[1::2]))
        self.zone_polygon = Polygon(coords)

        self.get_logger().info(
            f"Zone '{self.zone_name}' loaded | max_speed={self.max_speed} m/s | "
            f"vertices={coords}"
        )

        # Robot pose — None until first /amcl_pose message arrives
        self._robot_pose: Point | None = None
        self._pose_received = False

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            10
        )
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        # Publisher
        self.safe_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        self.get_logger().info('SpeedZoneLimiter node is ready.')

    # ------------------------------------------------------------------ #
    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._robot_pose = Point(x, y)
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(f'First pose received: ({x:.2f}, {y:.2f})')

    def _cmd_vel_callback(self, msg: Twist):
        safe_msg = Twist()
        safe_msg.angular = msg.angular  # angular velocity always passes through

        if not self._pose_received:
            # Haven't got a pose yet — pass velocity through unchanged
            safe_msg.linear = msg.linear
            self.safe_pub.publish(safe_msg)
            return

        in_zone = self.zone_polygon.contains(self._robot_pose)

        if in_zone:
            capped = min(msg.linear.x, self.max_speed)
            # Never allow negative linear.x to be un-capped while in zone
            safe_msg.linear.x = capped if msg.linear.x >= 0 else msg.linear.x
            safe_msg.linear.y = msg.linear.y
            safe_msg.linear.z = msg.linear.z
            if msg.linear.x > self.max_speed:
                self.get_logger().debug(
                    f'In zone "{self.zone_name}": capped {msg.linear.x:.3f} → {capped:.3f} m/s'
                )
        else:
            safe_msg.linear = msg.linear

        self.safe_pub.publish(safe_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedZoneLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
