import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from nav2_toolkit.status_line import emit
# Buffer：TF 缓存 + 查询引擎（存 /tf、/tf_static 进来的变换，并提供 lookup 查询）
# TransformListener：订阅 /tf、/tf_static，把收到的变换“灌进”Buffer（listener 负责填充，Buffer 负责查询）

class TfGuard(Node):
    def __init__(self):
        super().__init__("w2_d5_tf_guard")

        # MVP：两段链路写死（w2_d6 再做可配置）
        self.parent1 = "map"
        self.child1 = "odom"
        self.parent2 = "odom"
        self.child2 = "base_link"

        self.declare_parameter("timeout_sec", 0.2)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.buf = Buffer()
        # listener 第二个参数传 self：因为 self 是 Node 实例（继承自 Node），listener 需要用它来创建订阅 /tf、/tf_static
        # 这不是“指针乱用”，就是把 Node 句柄交给 listener 用；listener 收到数据后写入 self.buf
        self.listener = TransformListener(self.buf, self)

        # create_timer(period_sec, callback)：每 period_sec 秒触发一次 callback
        # 返回的是 Timer 对象，把它保存到 self.timer 是为了保活（避免被垃圾回收导致定时器失效）
        self.timer = self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            f"started: timeout_sec={self.timeout_sec} "
            f"check1={self.parent1}->{self.child1} check2={self.parent2}->{self.child2}"
        )

    def _check(self, parent: str, child: str):
        try:
            # lookup_transform(target_frame, source_frame, time, timeout=Duration)
            # 语义：查询 child 在 parent 下的变换（parent <- child）
            # rclpy.time.Time()：这里用于“请求最新可用的变换”（健康检查只关心链路是否活着）
            # timeout=Duration(seconds=...)：最多等待这么久让 buffer 里出现该变换；等不到就抛 TransformException
            t = self.buf.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.timeout_sec),
            )
            return True, t
        except TransformException as e:
            # 查不到：可能是 TF 链断、frame 名不存在、时间体系(use_sim_time)不一致、或启动初期数据还没进 buffer
            # 返回 (False, 错误原因字符串)，让上层稳定输出 FAIL 而不是让节点崩溃
            return False, str(e)

    def _on_timer(self):
        ok1, detail1 = self._check(self.parent1, self.child1)
        ok2, detail2 = self._check(self.parent2, self.child2)
        # _check 返回 (ok, detail)
        # ok=True  -> detail 是 TransformStamped（变换数据）
        # ok=False -> detail 是错误字符串（异常信息）

        # 输出一行状态（字段齐）
        # 规则：两段都 OK 才 OK；否则 FAIL
        if ok1 and ok2:
            emit(
                tool="tf_guard",
                level="OK",
                status="OK",
                reason="tf_ok",
                parent1=self.parent1, child1=self.child1,
                parent2=self.parent2, child2=self.child2,
                timeout_sec=f"{self.timeout_sec:.3f}",
            )
            # emit(tool/level/status/reason, **fields)：把必选字段 + 额外字段拼成一行 key=value 并打印（不是“自动调取”，是这里显式传入）
        else:
            # 哪段失败写清楚（ok1/ok2 表示各段是否查到）
            reason = "tf_missing"
            emit(
                tool="tf_guard",
                level="FAIL",
                status="FAIL",
                reason=reason,
                parent1=self.parent1, child1=self.child1,
                ok1=str(ok1),
                parent2=self.parent2, child2=self.child2,
                ok2=str(ok2),
                timeout_sec=f"{self.timeout_sec:.3f}",
            )
            # MVP 目前没把 detail1/detail2 打出来；需要更强排障时可加 detail1/detail2（建议截断长度避免日志爆炸）


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