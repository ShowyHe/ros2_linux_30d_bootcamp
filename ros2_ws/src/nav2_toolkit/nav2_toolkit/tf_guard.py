import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from nav2_toolkit.status_line import emit

class TfGuard(Node):
    def __init__(self):
        super().__init__("w2_d5_tf_guard")

        self.declare_parameter("parent_frame","/map")
        self.declare_parameter("child_frame","/odom")
        self.parent_frame=str(self.get_parameter("parent_frame").value)
        self.child_frame=str(self.get_parameter("child_frame").value)

        self.declare_parameter("timeout_sec", 0.2)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.timer = self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            f"started: timeout_sec={self.timeout_sec} "
            f"check1={self.parent_frame}->{self.child_frame}"
        )

    def _check(self, parent: str, child: str):
        try:
            t = self.buf.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.timeout_sec),
            )
            return True, t
        except TransformException as e:
            return False, str(e)

    def _on_timer(self):
        ok, detail = self._check(self.parent_frame, self.child_frame)
        if ok:
            emit(
                tool="tf_guard",
                level="OK",
                status="OK",
                reason="tf_ok",
                parent=self.parent_frame, child=self.child_frame,
                timeout_sec=f"{self.timeout_sec:.3f}",
            )
        else:
            reason = "tf_missing"
            emit(
                tool="tf_guard",
                level="FAIL",
                status="FAIL",
                reason=reason,
                parent=self.parent_frame, child=self.child_frame,
                ok=str(ok),
                timeout_sec=f"{self.timeout_sec:.3f}",
            )

def main():
    rclpy.init()
    node = TfGuard()
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