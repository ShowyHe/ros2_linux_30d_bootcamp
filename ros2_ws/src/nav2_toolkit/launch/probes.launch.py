from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  #substitution替换
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration("ns")  #configuration配置
    topic_odom = LaunchConfiguration("topic_odom")
    print_rate = LaunchConfiguration("print_rate")
    min_rate_threshold = LaunchConfiguration("min_rate_threshold")
    stale_timeout_sec = LaunchConfiguration("stale_timeout_sec")
    #在 Node 启动时，把这些配置项的当前值拿来用

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="r1"),
        DeclareLaunchArgument("topic_odom", default_value="/odom"),
        DeclareLaunchArgument("print_rate", default_value="1.0"),
        DeclareLaunchArgument("min_rate_threshold", default_value="10.0"),
        DeclareLaunchArgument("stale_timeout_sec", default_value="1.0"),
        #声明“有哪些可配置项+默认值”

        Node(
            package="nav2_toolkit",
            executable="topic_probe",
            name="w2_d1_topic_probe",
            namespace=ns,
            output="screen",
            parameters=[{
                "topic_odom": topic_odom,
                "print_rate": print_rate,
                "min_rate_threshold": min_rate_threshold,
                "stale_timeout_sec": stale_timeout_sec,  #过期的超时
            #Node 用这些值设置 namespace 和 ROS 参数
            }],
        ),
    ])
#在LaunchDescription([...]) 里声明了一堆东西（动作/配置），
#launch 系统在运行时会把 LaunchConfiguration("xxx") 这种“取值表达式”解析成实际值（默认值或命令行覆盖值）
#然后用解析出来的值去启动 Node，并把参数传进去
