import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class TopicProbe(Node):
    def __init__(self):
        super().__init__("w2_d1_topic_probe")

        self.declare_parameter("topic_odom","/odom")
        self.declare_parameter("print_rate",1.0)
        self.topic_odom=str(self.get_parameter("topic_odom").value)
        self.print_rate=float(self.get_parameter("print_rate").value)

        self.count=0
        self.last_lin_x=None
        self.last_ang_z=None
        self.last_tick=self.get_clock().now()

        self.sub=self.create_subscription(Odometry,self.topic_odom,self._on_odom,10)

        period=1.0/max(self.print_rate,0.1)
        self.timer=self.create_timer(period,self._on_timer)

        self.get_logger().info(f"stared: print_rate={self.print_rate}")

    def _on_odom(self,msg:Odometry):
        self.count+=1
        self.last_lin_x=float(msg.twist.twist.linear.x)
        self.last_ang_z=float(msg.twist.twist.angular.z)

    def _on_timer(self):
        now=self.get_clock().now()
        elapsed=(now-self.last_tick).nanoseconds/1e9 
        rate_hz=self.count/elapsed if elapsed > 0 else 0.0
        if self.last_lin_x is None:
            self.get_logger().warn("warning,velocity not received")
        else:
            self.get_logger().info(
                f"lin_x={self.last_lin_x:.3f},ang_z={self.last_ang_z:.3f},rate_hz={rate_hz:.2f}"
            )

        self.last_tick=now
        self.count=0

def main():
    rclpy.init()
    node=TopicProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()