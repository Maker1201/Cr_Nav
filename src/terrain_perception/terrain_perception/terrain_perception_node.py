import math
import statistics
import threading
from typing import Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

Point = Tuple[float, float, float]


def _quat_to_matrix(x: float, y: float, z: float, w: float):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _transform_point(point: Point, transform) -> Point:
    t = transform.transform.translation
    q = transform.transform.rotation
    matrix = _quat_to_matrix(q.x, q.y, q.z, q.w)
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + t.x,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + t.y,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + t.z,
    )


class TerrainPerception(Node):
    def __init__(self):
        super().__init__('terrain_perception')

        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.front_topic = self.declare_parameter('front_points_topic', '/orbbec_front/depth/points').value
        self.rear_topic = self.declare_parameter('rear_points_topic', '/orbbec_rear/depth/points').value
        scan_topic = self.declare_parameter('terrain_scan_topic', '/terrain_scan').value
        obstacles_topic = self.declare_parameter('terrain_obstacles_topic', '/terrain_obstacles').value

        self.max_drop = float(self.declare_parameter('max_drop_depth_m', 0.05).value)
        self.max_step = float(self.declare_parameter('max_step_height_m', 0.04).value)
        self.max_slope_deg = float(self.declare_parameter('max_slope_deg', 10.0).value)
        self.min_obstacle_points = int(self.declare_parameter('min_obstacle_points', 4).value)

        self.front_x_min = float(self.declare_parameter('front_roi_x_min_m', 0.05).value)
        self.front_x_max = float(self.declare_parameter('front_roi_x_max_m', 1.40).value)
        self.rear_x_min = float(self.declare_parameter('rear_roi_x_min_m', -1.40).value)
        self.rear_x_max = float(self.declare_parameter('rear_roi_x_max_m', -0.05).value)
        self.roi_y_abs = float(self.declare_parameter('roi_y_abs_m', 0.55).value)
        self.roi_z_min = float(self.declare_parameter('roi_z_min_m', -0.35).value)
        self.roi_z_max = float(self.declare_parameter('roi_z_max_m', 0.85).value)

        self.angle_min = float(self.declare_parameter('scan_angle_min_rad', -math.pi).value)
        self.angle_max = float(self.declare_parameter('scan_angle_max_rad', math.pi).value)
        self.angle_increment = float(self.declare_parameter('scan_angle_increment_rad', 0.01745).value)
        self.range_min = float(self.declare_parameter('scan_range_min_m', 0.05).value)
        self.range_max = float(self.declare_parameter('scan_range_max_m', 3.0).value)
        publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 10.0).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.latest_front: List[Point] = []
        self.latest_rear: List[Point] = []
        self.lock = threading.Lock()

        self.create_subscription(PointCloud2, self.front_topic, self._front_callback, sensor_qos)
        self.create_subscription(PointCloud2, self.rear_topic, self._rear_callback, sensor_qos)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 5)
        self.obstacle_pub = self.create_publisher(PointCloud2, obstacles_topic, 5)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._publish)

        self.get_logger().info(
            f'Terrain perception started: front={self.front_topic}, rear={self.rear_topic}, base={self.base_frame}')

    def _front_callback(self, msg: PointCloud2):
        points = self._cloud_to_base_points(msg, front=True)
        with self.lock:
            self.latest_front = points

    def _rear_callback(self, msg: PointCloud2):
        points = self._cloud_to_base_points(msg, front=False)
        with self.lock:
            self.latest_rear = points

    def _cloud_to_base_points(self, msg: PointCloud2, front: bool) -> List[Point]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed from {msg.header.frame_id} to {self.base_frame}: {exc}')
            return []

        output = []
        for raw in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            point = _transform_point((float(raw[0]), float(raw[1]), float(raw[2])), transform)
            if self._in_roi(point, front):
                output.append(point)
        return output

    def _in_roi(self, point: Point, front: bool) -> bool:
        x, y, z = point
        if abs(y) > self.roi_y_abs or z < self.roi_z_min or z > self.roi_z_max:
            return False
        if front:
            return self.front_x_min <= x <= self.front_x_max
        return self.rear_x_min <= x <= self.rear_x_max

    def _hazards_from_points(self, points: Sequence[Point]) -> List[Point]:
        if len(points) < self.min_obstacle_points:
            return []

        z_values = [p[2] for p in points]
        ground_z = statistics.median(z_values)
        slope_limit = math.tan(math.radians(self.max_slope_deg))
        hazards = []

        for x, y, z in points:
            dz = z - ground_z
            distance = max(math.hypot(x, y), 0.05)
            slope = abs(dz) / distance
            if dz > self.max_step or dz < -self.max_drop or slope > slope_limit:
                hazards.append((x, y, z))
        return hazards if len(hazards) >= self.min_obstacle_points else []

    def _publish(self):
        with self.lock:
            front = list(self.latest_front)
            rear = list(self.latest_rear)

        hazards = self._hazards_from_points(front) + self._hazards_from_points(rear)
        now = self.get_clock().now().to_msg()
        self._publish_scan(hazards, now)
        self._publish_obstacles(hazards, now)

    def _publish_scan(self, hazards: Sequence[Point], stamp):
        count = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment))
        ranges = [math.inf] * count
        for x, y, _ in hazards:
            distance = math.hypot(x, y)
            if distance < self.range_min or distance > self.range_max:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle >= self.angle_max:
                continue
            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < count and distance < ranges[index]:
                ranges[index] = distance

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def _publish_obstacles(self, hazards: Sequence[Point], stamp):
        header = LaserScan().header
        header.stamp = stamp
        header.frame_id = self.base_frame
        cloud = point_cloud2.create_cloud_xyz32(header, list(hazards))
        self.obstacle_pub.publish(cloud)


def main(args: Optional[Iterable[str]] = None):
    rclpy.init(args=args)
    node = TerrainPerception()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
