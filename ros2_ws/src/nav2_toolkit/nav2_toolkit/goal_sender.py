# client 先等待 server 就绪（wait_for_server），
# 再发送 goal 并判断 server 是否接单（goal_handle.accepted）；
# 如果接单，再等待该导航任务的最终结果（get_result_async + spin_until_future_complete）。
# 这两个 spin_until_future_complete 都是在让节点处理 ROS 通信事件，直到对应的 Future 完成。

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from nav2_toolkit.status_line import emit

def yaw_to_quat(yaw: float):
    # 简单 yaw -> quaternion（roll/pitch=0）
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


# 结果状态码 -> 名称（方便日志可读）
STATUS_NAME = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__("w2_d7_goal_sender")

        # --- parameters ---
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("timeout_sec", 120.0)
        self.declare_parameter("server_timeout_sec", 3.0)
        self.declare_parameter("action_name", "/navigate_to_pose")

        self.frame_id = str(self.get_parameter("frame_id").value).lstrip("/")
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.server_timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.action_name = str(self.get_parameter("action_name").value)

        self.client = ActionClient(self, NavigateToPose, self.action_name)
        # 创建 Action 客户端
        # 当前节点 self 作为客户端，使用 NavigateToPose 协议，连接 /navigate_to_pose 这个 action server 接口

        self.get_logger().info(
            f"started: frame_id={self.frame_id} x={self.x} y={self.y} yaw={self.yaw} "
            f"timeout_sec={self.timeout_sec} server_timeout_sec={self.server_timeout_sec} "
            f"action_name={self.action_name}"
        )

    def send_once(self) -> int:
        t0 = time.time()

        # 1) 等 server
        if not self.client.wait_for_server(timeout_sec=self.server_timeout_sec):
            # 等待 action server 就绪（可用），不是“收消息”
            emit(
                tool="goal_sender", level="FAIL", status="FAIL", reason="server_not_ready",
                action=self.action_name, server_timeout_sec=f"{self.server_timeout_sec:.1f}"
            )
            return 2

        # 2) 构造 goal
        goal_msg = NavigateToPose.Goal()
        # 按 NavigateToPose 协议构造 goal（目标位姿）
        # 这里填的是终点，不是起点；起点由当前定位/TF决定
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y

        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        emit(
            tool="goal_sender", level="OK", status="OK", reason="goal_sent",
            action=self.action_name, frame=self.frame_id,
            x=f"{self.x:.3f}", y=f"{self.y:.3f}", yaw=f"{self.yaw:.3f}"
        )

        # 3) 发送 + 等“接单回执”
        send_future = self.client.send_goal_async(goal_msg)
        # 异步发送 goal，请求 server 接单；返回的是“接单回执”的 Future，不是最终导航结果
        rclpy.spin_until_future_complete(self, send_future)
        # 让节点处理通信事件，直到 send_future 完成

        if not send_future.done():
            emit(
                tool="goal_sender", level="FAIL", status="FAIL", reason="send_goal_future_not_done",
                action=self.action_name
            )
            return 5

        goal_handle = send_future.result()
        # 完成后取出 goal_handle（目标句柄）

        if goal_handle is None:
            emit(
                tool="goal_sender", level="FAIL", status="FAIL", reason="goal_handle_none",
                action=self.action_name
            )
            return 6

        if not goal_handle.accepted:
            emit(
                tool="goal_sender", level="FAIL", status="FAIL", reason="goal_rejected",
                action=self.action_name
            )
            return 3

        emit(
            tool="goal_sender", level="OK", status="OK", reason="goal_accepted",
            action=self.action_name
        )

        # 4) 等最终结果
        result_future = goal_handle.get_result_async()
        # 请求该 goal 的最终结果（result/status），返回“完工结果”的 Future

        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
        # spin 等待导航任务执行完成（服务端在执行），最多等待 timeout_sec 秒

        if not result_future.done():
            elapsed = time.time() - t0
            emit(
                tool="goal_sender", level="FAIL", status="FAIL", reason="result_timeout",
                action=self.action_name,
                timeout_sec=f"{self.timeout_sec:.1f}",
                time_sec=f"{elapsed:.3f}"
            )
            return 4
        # 在超时时间内没有等到最终结果，判定为 result_timeout

        wrapped_result = result_future.result()
        if wrapped_result is None:
            elapsed = time.time() - t0
            emit(
                tool="goal_sender", level="FAIL", status="FAIL", reason="result_future_none",
                action=self.action_name,
                time_sec=f"{elapsed:.3f}"
            )
            return 7

        status = int(wrapped_result.status)
        # result = wrapped_result.result   # 目前先不用，保留位子给后续扩展
        status_name = STATUS_NAME.get(status, f"UNKNOWN_{status}")
        elapsed = time.time() - t0
        # 从结果 Future 中取出最终响应：包含动作状态码 status 和动作结果载荷 result

        # 关键修正：只有 SUCCEEDED 才记 OK；ABORTED/CANCELED 要记 FAIL
        if status == GoalStatus.STATUS_SUCCEEDED:
            emit(
                tool="goal_sender", level="OK", status="OK", reason="result_received",
                action=self.action_name,
                result_status=str(status),
                result_status_name=status_name,
                time_sec=f"{elapsed:.3f}"
            )
            return 0

        emit(
            tool="goal_sender", level="FAIL", status="FAIL", reason="result_received",
            action=self.action_name,
            result_status=str(status),
            result_status_name=status_name,
            time_sec=f"{elapsed:.3f}"
        )
        return 8


def main():
    rclpy.init()
    node = Nav2GoalSender()
    try:
        rc = node.send_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()