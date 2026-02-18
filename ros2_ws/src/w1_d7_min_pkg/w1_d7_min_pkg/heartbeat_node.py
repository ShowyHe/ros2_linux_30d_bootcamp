import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('w1_d7_heartbeat')

        self.declare_parameter('rate_hz',1.0)
        rate_hz=float(self.get_parameter('rate_hz').value)
        period=1.0/max(rate_hz,0.1)

        self.pub=self.create_publisher(String,"/w1_d7/heartbeat",10)
        self.timer=self.create_timer(period,self._on_timer)
        self.count=0

        self.get_logger().info(f"started:rate_hz={rate_hz}")
    
    def _on_timer(self):
        msg=String()   #为什么msg不用self？
        msg.data=f"time count={self.count}"
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count+=1

def main():
    rclpy.init()
    node=HeartbeatNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__=="__main__":
    main()
