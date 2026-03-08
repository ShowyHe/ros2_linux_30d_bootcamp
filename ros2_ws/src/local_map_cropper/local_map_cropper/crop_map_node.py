#!/usr/bin/env python3
import math
import os
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException


class CropMapNode(Node):
    def __init__(self):
        super().__init__('crop_map_node')

        # 参数
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('crop_size_m', 8.0)
        self.declare_parameter(
            'output_dir',
            os.path.expanduser('~/ros2_linux_30d_bootcamp/data/local_map_cropper')
        )
        self.declare_parameter('output_stem', 'cropped_map')
        self.declare_parameter('save_once', True)

        self.map_topic = self.get_parameter('map_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.crop_size_m = float(self.get_parameter('crop_size_m').value)
        self.output_dir = self.get_parameter('output_dir').value
        self.output_stem = self.get_parameter('output_stem').value
        self.save_once = bool(self.get_parameter('save_once').value)

        os.makedirs(self.output_dir, exist_ok=True)

        self.full_map = None
        self.saved = False

        # /map 常常是 transient_local，晚启动订阅者也要能收到
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('crop_map_node started')

    def map_callback(self, msg: OccupancyGrid):
        self.full_map = msg
        self.get_logger().info(
            f'Received map: width={msg.info.width}, height={msg.info.height}, '
            f'resolution={msg.info.resolution}'
        )

    def timer_callback(self):
        if self.save_once and self.saved:
            return

        if self.full_map is None:
            self.get_logger().info('Waiting for /map ...')
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )
        except TransformException as e:
            self.get_logger().warn(f'Failed to lookup TF {self.global_frame}->{self.robot_frame}: {e}')
            return

        robot_x = tf_msg.transform.translation.x
        robot_y = tf_msg.transform.translation.y

        self.get_logger().info(f'Robot pose in map: x={robot_x:.3f}, y={robot_y:.3f}')

        ok = self.crop_and_save(robot_x, robot_y)
        if ok:
            self.saved = True
            self.get_logger().info('Crop and save finished successfully.')

    def crop_and_save(self, robot_x: float, robot_y: float) -> bool:
        info = self.full_map.info
        resolution = info.resolution
        width = info.width
        height = info.height
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        data = list(self.full_map.data)

        if resolution <= 0.0:
            self.get_logger().error('Invalid map resolution.')
            return False

        # 世界坐标 -> 栅格坐标
        col = int((robot_x - origin_x) / resolution)
        row = int((robot_y - origin_y) / resolution)

        if not (0 <= col < width and 0 <= row < height):
            self.get_logger().error(
                f'Robot grid index out of map range: row={row}, col={col}, '
                f'map_height={height}, map_width={width}'
            )
            return False

        crop_cells = max(1, int(self.crop_size_m / resolution))
        half = crop_cells // 2

        min_col = max(0, col - half)
        max_col = min(width, col + half)
        min_row = max(0, row - half)
        max_row = min(height, row + half)

        new_width = max_col - min_col
        new_height = max_row - min_row

        if new_width <= 0 or new_height <= 0:
            self.get_logger().error('Invalid crop size after clipping.')
            return False

        cropped_data = []
        for r in range(min_row, max_row):
            base = r * width
            for c in range(min_col, max_col):
                cropped_data.append(data[base + c])

        new_origin_x = origin_x + min_col * resolution
        new_origin_y = origin_y + min_row * resolution

        pgm_path = os.path.join(self.output_dir, f'{self.output_stem}.pgm')
        yaml_path = os.path.join(self.output_dir, f'{self.output_stem}.yaml')

        self.write_pgm(pgm_path, new_width, new_height, cropped_data)
        self.write_yaml(
            yaml_path=yaml_path,
            image_name=f'{self.output_stem}.pgm',
            resolution=resolution,
            origin_x=new_origin_x,
            origin_y=new_origin_y
        )

        self.get_logger().info(
            f'Saved cropped map:\n'
            f'  pgm : {pgm_path}\n'
            f'  yaml: {yaml_path}\n'
            f'  crop width={new_width}, height={new_height}\n'
            f'  new origin=({new_origin_x:.3f}, {new_origin_y:.3f})'
        )
        return True

    def occ_to_pgm_value(self, occ: int) -> int:
        # map_server 常见约定：
        # 0   (free)     -> 白色(254)
        # 100 (occupied) -> 黑色(0)
        # -1  (unknown)  -> 灰色(205)
        if occ == -1:
            return 205
        if occ >= 65:
            return 0
        if occ <= 25:
            return 254
        return 205

    def write_pgm(self, pgm_path: str, width: int, height: int, cropped_data: List[int]):
        # PGM 按“图像从上到下”写；OccupancyGrid 的 row=0 在地图下边，
        # 所以这里要倒着写行，否则图会上下翻转。
        with open(pgm_path, 'wb') as f:
            header = f'P5\n{width} {height}\n255\n'
            f.write(header.encode('ascii'))

            for image_row in range(height - 1, -1, -1):
                for c in range(width):
                    idx = image_row * width + c
                    pixel = self.occ_to_pgm_value(cropped_data[idx])
                    f.write(bytes([pixel]))

    def write_yaml(
        self,
        yaml_path: str,
        image_name: str,
        resolution: float,
        origin_x: float,
        origin_y: float
    ):
        content = (
            f'image: {image_name}\n'
            f'mode: trinary\n'
            f'resolution: {resolution}\n'
            f'origin: [{origin_x}, {origin_y}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
        )
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(content)


def main(args=None):
    rclpy.init(args=args)
    node = CropMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()