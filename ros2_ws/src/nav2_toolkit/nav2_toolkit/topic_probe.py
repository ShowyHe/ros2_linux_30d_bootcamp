import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# NEW: status line output
from nav2_toolkit.status_line import emit


class TopicProbe(Node):
    def __init__(self):
        super().__init__("w2_d1_topic_probe")

        # --- parameters (must declare + use) ---
        self.declare_parameter("topic_odom", "/odom")
        self.declare_parameter("print_rate", 1.0)

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

        self.last_tick = self.get_clock().now()
        self.last_msg_time = None

        period = 1.0 / max(self.print_rate, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        # 启动信息保留 logger（一次性的，不影响契约）
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

        # 统一输出字段（建议键名稳定）
        common = dict(
            topic=self.topic_odom,
            rate_hz=f"{rate_hz:.3f}",
            threshold=f"{self.min_rate_threshold:.3f}",
            stale_sec=f"{self.stale_timeout_sec:.3f}",
        )

        # 2) stale / no-msg detection
        if self.last_msg_time is None:
            emit(
                tool="topic_probe",
                level="FAIL",
                status="FAIL",
                reason="stale_no_msg",
                **common
            )
        else:
            since_last = (now - self.last_msg_time).nanoseconds / 1e9

            # STALE -> FAIL
            if since_last > self.stale_timeout_sec:
                emit(
                    tool="topic_probe",
                    level="FAIL",
                    status="FAIL",
                    reason="stale_timeout",
                    stale_age_sec=f"{since_last:.3f}",
                    **common
                )
            else:
                # 3) low-rate detection -> WARN
                if rate_hz < self.min_rate_threshold:
                    emit(
                        tool="topic_probe",
                        level="WARN",
                        status="WARN",
                        reason="low_rate",
                        lin_x=f"{(self.last_lin_x if self.last_lin_x is not None else 0.0):.3e}",
                        ang_z=f"{(self.last_ang_z if self.last_ang_z is not None else 0.0):.3e}",
                        **common
                    )
                else:
                    emit(
                        tool="topic_probe",
                        level="OK",
                        status="OK",
                        reason="rate_ok",
                        lin_x=f"{(self.last_lin_x if self.last_lin_x is not None else 0.0):.3e}",
                        ang_z=f"{(self.last_ang_z if self.last_ang_z is not None else 0.0):.3e}",
                        **common
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()