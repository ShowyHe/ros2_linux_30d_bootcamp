# docs/w2_d5_tf_guard_readme.md（Week2 Day5 / Global Day12）

## 今日目标
- 我写了一个 tf_guard（MVP），用 tf2_ros Buffer/Listener 查询 TF
- 每秒输出一行状态（沿用 w2_d3p5 的 status line 契约）
- 我能制造一次 FAIL，并用日志作为硬证据（不是“我觉得 TF 不对”）

---

## 代码（防止明天修改之后没有痕迹）
import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from nav2_toolkit.status_line import emit

class TfGuard(Node):
    def __init__(self):
        super().__init__("w2_d5_tf_guard")

        self.parent1 = "map"
        self.child1 = "odom"
        self.parent2 = "odom"
        self.child2 = "base_link"

        self.declare_parameter("timeout_sec", 0.2)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.buf = Buffer()
        tf_static
        self.listener = TransformListener(self.buf, self)
        self.timer = self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            f"started: timeout_sec={self.timeout_sec} "
            f"check1={self.parent1}->{self.child1} check2={self.parent2}->{self.child2}"
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
        ok1, detail1 = self._check(self.parent1, self.child1)
        ok2, detail2 = self._check(self.parent2, self.child2)
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
        else:
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

## 工具行为定义（MVP）
默认检查两段链路：
- check1：map -> odom
- check2：odom -> base_link

判定逻辑（MVP 口径）：
- 两段都查到 transform：level=OK reason=tf_ok
- 任意一段查不到/超时：level=FAIL reason=tf_missing，并输出 ok1/ok2

---

## 运行方式（落盘证据）
建议用 tee 落盘：
    cd ~/ros2_linux_30d_bootcamp
    mkdir -p logs
    : > logs/w2_d5_tf_guard.log
    ros2 run nav2_toolkit tf_guard 2>&1 | tee -a logs/w2_d5_tf_guard.log

---

## 证据样例（从今天真实输出摘）
### FAIL（我故意把 child2 改成不存在的 frame：no_such_frame）
    ts=2026-02-23T18:24:52+08:00 tool=tf_guard level=FAIL status=FAIL reason=tf_missing parent1=map child1=odom ok1=True parent2=odom child2=no_such_frame ok2=False timeout_sec=0.200

结论：
- map->odom 存在（ok1=True）
- odom->no_such_frame 不存在（ok2=False）
- 所以工具稳定输出 FAIL（reason=tf_missing），这是我今天的硬证据

---

## 今日踩坑
1) 我一开始想用 `--ros-args -p parent1:=/scan` 来制造 FAIL，这个思路不对：
   - `/scan` 是 topic，不是 TF frame
   - TF 用的是 frame_id/child_frame_id（如 map/odom/base_link/base_scan）

2) 我今天用“改成不存在的 frame 名”制造 FAIL 是最稳的：
   - 不依赖 Gazebo/Nav2 是否启动
   - 结果可复现、证据可落盘