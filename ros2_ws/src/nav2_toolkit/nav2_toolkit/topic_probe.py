import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TopicProbe(Node):
    def __init__(self):
        super().__init__("w2_d1_topic_probe")

        # --- parameters (must declare + use) ---
        self.declare_parameter("topic_odom", "/odom")
        self.declare_parameter("print_rate", 1.0)

        # w2_d2 added
        self.declare_parameter("min_rate_threshold", 10.0)   # Hz
        self.declare_parameter("stale_timeout_sec", 1.0)     # seconds

        self.topic_odom = str(self.get_parameter("topic_odom").value)
        self.print_rate = float(self.get_parameter("print_rate").value)
        self.min_rate_threshold = float(self.get_parameter("min_rate_threshold").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)

        # --- subscription ---
        self.sub = self.create_subscription(Odometry, self.topic_odom, self._on_odom, 10)

        # --- state ---
        self.count = 0
        self.last_lin_x = None
        self.last_ang_z = None

        self.last_tick = self.get_clock().now()  # for rate estimation per timer window
        self.last_msg_time = None                # for stale detection (None => never received)

        period = 1.0 / max(self.print_rate, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"started: topic_odom={self.topic_odom}, print_rate={self.print_rate}, "
            f"min_rate_threshold={self.min_rate_threshold}, stale_timeout_sec={self.stale_timeout_sec}"
        )

    def _on_odom(self, msg: Odometry):
        self.count += 1
        self.last_lin_x = float(msg.twist.twist.linear.x)
        self.last_ang_z = float(msg.twist.twist.angular.z)
        self.last_msg_time = self.get_clock().now()

    def _on_timer(self):
        now = self.get_clock().now()

        # 1) estimate rate over the last timer interval
        elapsed = (now - self.last_tick).nanoseconds / 1e9
        rate_hz = (self.count / elapsed) if elapsed > 0.0 else 0.0

        # 2) stale / no-msg detection
        if self.last_msg_time is None:
            self.get_logger().warn(f"NO_MSG: never received odom on {self.topic_odom}")
        else:
            since_last = (now - self.last_msg_time).nanoseconds / 1e9
            if since_last > self.stale_timeout_sec:
                self.get_logger().warn(
                    f"STALE: no odom for {since_last:.2f}s (timeout={self.stale_timeout_sec:.2f}s) "
                    f"on {self.topic_odom}"
                )
            else:
                # 3) low-rate detection (only meaningful when not stale)
                if rate_hz < self.min_rate_threshold:
                    self.get_logger().warn(
                        f"LOW_RATE: rate_hz={rate_hz:.2f} < {self.min_rate_threshold:.2f} "
                        f"lin_x={self.last_lin_x:.3e} ang_z={self.last_ang_z:.3e}"
                    )
                else:
                    self.get_logger().info(
                        f"OK: rate_hz={rate_hz:.2f} lin_x={self.last_lin_x:.3e} ang_z={self.last_ang_z:.3e}"
                    )

        # reset window
        self.last_tick = now
        self.count = 0


def main():
    rclpy.init()
    node = TopicProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # avoid "rcl_shutdown already called"
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
