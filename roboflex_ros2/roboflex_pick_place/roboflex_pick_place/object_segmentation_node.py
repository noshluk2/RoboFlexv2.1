#!/usr/bin/env python3

import copy
import math
from collections import defaultdict, deque
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class ObjectSegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__("object_segmentation_node")

        self.declare_parameter("input_topic", "/camera/depth/color/points")
        self.declare_parameter("output_topic", "/camera/depth/color/points_segmented")
        self.declare_parameter("target_frame", "")
        self.declare_parameter("min_x", 0.05)
        self.declare_parameter("max_x", 0.45)
        self.declare_parameter("min_y", -0.30)
        self.declare_parameter("max_y", 0.30)
        self.declare_parameter("min_z", -0.05)
        self.declare_parameter("max_z", 0.40)
        self.declare_parameter("filter_above_table", True)
        self.declare_parameter("table_z", 0.0)
        self.declare_parameter("table_clearance_m", 0.010)
        self.declare_parameter("publish_empty_cloud", True)
        self.declare_parameter("publish_bounding_boxes", True)
        self.declare_parameter("bbox_topic", "/segmented_objects/bounding_boxes")
        self.declare_parameter("cluster_tolerance_m", 0.025)
        self.declare_parameter("min_cluster_points", 80)
        self.declare_parameter("max_object_size_x", 0.18)
        self.declare_parameter("max_object_size_y", 0.18)
        self.declare_parameter("max_object_size_z", 0.18)
        self.declare_parameter("min_object_size_x", 0.01)
        self.declare_parameter("min_object_size_y", 0.01)
        self.declare_parameter("min_object_size_z", 0.005)
        self.declare_parameter("max_bounding_boxes", 6)
        self.declare_parameter("log_interval_sec", 2.0)

        self.input_topic_ = self.get_parameter("input_topic").value
        self.output_topic_ = self.get_parameter("output_topic").value
        self.target_frame_ = self.get_parameter("target_frame").value
        self.min_x_ = float(self.get_parameter("min_x").value)
        self.max_x_ = float(self.get_parameter("max_x").value)
        self.min_y_ = float(self.get_parameter("min_y").value)
        self.max_y_ = float(self.get_parameter("max_y").value)
        self.min_z_ = float(self.get_parameter("min_z").value)
        self.max_z_ = float(self.get_parameter("max_z").value)
        self.filter_above_table_ = bool(self.get_parameter("filter_above_table").value)
        self.table_z_ = float(self.get_parameter("table_z").value)
        self.table_clearance_m_ = max(0.0, float(self.get_parameter("table_clearance_m").value))
        self.publish_empty_cloud_ = bool(self.get_parameter("publish_empty_cloud").value)
        self.publish_bounding_boxes_ = bool(self.get_parameter("publish_bounding_boxes").value)
        self.bbox_topic_ = self.get_parameter("bbox_topic").value
        self.cluster_tolerance_m_ = max(0.005, float(self.get_parameter("cluster_tolerance_m").value))
        self.min_cluster_points_ = max(1, int(self.get_parameter("min_cluster_points").value))
        self.max_object_size_x_ = max(0.001, float(self.get_parameter("max_object_size_x").value))
        self.max_object_size_y_ = max(0.001, float(self.get_parameter("max_object_size_y").value))
        self.max_object_size_z_ = max(0.001, float(self.get_parameter("max_object_size_z").value))
        self.min_object_size_x_ = max(0.0, float(self.get_parameter("min_object_size_x").value))
        self.min_object_size_y_ = max(0.0, float(self.get_parameter("min_object_size_y").value))
        self.min_object_size_z_ = max(0.0, float(self.get_parameter("min_object_size_z").value))
        self.max_bounding_boxes_ = max(1, int(self.get_parameter("max_bounding_boxes").value))
        self.log_interval_ns_ = int(float(self.get_parameter("log_interval_sec").value) * 1e9)

        self.min_x_, self.max_x_ = self._normalize_bounds("x", self.min_x_, self.max_x_)
        self.min_y_, self.max_y_ = self._normalize_bounds("y", self.min_y_, self.max_y_)
        self.min_z_, self.max_z_ = self._normalize_bounds("z", self.min_z_, self.max_z_)
        self.min_object_size_x_, self.max_object_size_x_ = self._normalize_bounds(
            "object_size_x", self.min_object_size_x_, self.max_object_size_x_
        )
        self.min_object_size_y_, self.max_object_size_y_ = self._normalize_bounds(
            "object_size_y", self.min_object_size_y_, self.max_object_size_y_
        )
        self.min_object_size_z_, self.max_object_size_z_ = self._normalize_bounds(
            "object_size_z", self.min_object_size_z_, self.max_object_size_z_
        )

        self.tf_buffer_: Optional[Buffer] = None
        self.tf_listener_: Optional[TransformListener] = None
        if self.target_frame_:
            self.tf_buffer_ = Buffer()
            self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.last_warn_ns_ = 0
        self.last_info_ns_ = 0
        self.last_bbox_count_ = 0

        self.segmented_pub_ = self.create_publisher(PointCloud2, self.output_topic_, 10)
        self.bbox_pub_ = self.create_publisher(MarkerArray, self.bbox_topic_, 10)
        self.pointcloud_sub_ = self.create_subscription(
            PointCloud2,
            self.input_topic_,
            self._pointcloud_callback,
            10,
        )

        frame_text = self.target_frame_ if self.target_frame_ else "input pointcloud frame"
        table_text = (
            f"table_z={self.table_z_:.3f} clearance={self.table_clearance_m_:.3f}"
            if self.filter_above_table_
            else "table filter disabled"
        )
        bbox_text = (
            f"bbox_topic={self.bbox_topic_} size_limit=({self.max_object_size_x_:.3f}, "
            f"{self.max_object_size_y_:.3f}, {self.max_object_size_z_:.3f})"
            if self.publish_bounding_boxes_
            else "bbox publishing disabled"
        )
        self.get_logger().info(
            "Object segmentation started. "
            f"input={self.input_topic_} output={self.output_topic_} bounds_frame={frame_text} "
            f"bounds=([{self.min_x_:.3f}, {self.max_x_:.3f}], "
            f"[{self.min_y_:.3f}, {self.max_y_:.3f}], "
            f"[{self.min_z_:.3f}, {self.max_z_:.3f}]) "
            f"{table_text} {bbox_text}"
        )

    def _pointcloud_callback(self, cloud_msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds

        field_names = [field.name for field in cloud_msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            self._warn_throttled(now_ns, "PointCloud2 message is missing x/y/z fields")
            return

        x_idx = field_names.index("x")
        y_idx = field_names.index("y")
        z_idx = field_names.index("z")

        transform = None
        if self.target_frame_:
            if not cloud_msg.header.frame_id:
                self._warn_throttled(now_ns, "PointCloud2 header frame_id is empty")
                return
            if cloud_msg.header.frame_id != self.target_frame_:
                try:
                    transform = self.tf_buffer_.lookup_transform(
                        self.target_frame_,
                        cloud_msg.header.frame_id,
                        Time(),
                    )
                except TransformException as exc:
                    self._warn_throttled(
                        now_ns,
                        f"Missing TF from {cloud_msg.header.frame_id} to {self.target_frame_}: {exc}",
                    )
                    return

        segmented_points = []
        segmented_xyz = []
        total_points = 0
        kept_points = 0

        for point in point_cloud2.read_points(
            cloud_msg,
            field_names=field_names,
            skip_nans=True,
        ):
            total_points += 1

            px = float(point[x_idx])
            py = float(point[y_idx])
            pz = float(point[z_idx])

            if transform is not None:
                px, py, pz = self._transform_point(
                    px,
                    py,
                    pz,
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )

            if not self._is_inside_bounds(px, py, pz):
                continue

            if self.filter_above_table_ and pz <= (self.table_z_ + self.table_clearance_m_):
                continue

            point_values = list(point)
            if self.target_frame_:
                point_values[x_idx] = px
                point_values[y_idx] = py
                point_values[z_idx] = pz
            segmented_points.append(tuple(point_values))
            segmented_xyz.append((px, py, pz))
            kept_points += 1

        out_header = copy.copy(cloud_msg.header)
        if self.target_frame_:
            out_header.frame_id = self.target_frame_

        if kept_points == 0 and not self.publish_empty_cloud_:
            self._publish_bounding_boxes(out_header=out_header, points_xyz=[])
            return

        segmented_cloud = point_cloud2.create_cloud(out_header, cloud_msg.fields, segmented_points)
        segmented_cloud.is_bigendian = cloud_msg.is_bigendian
        segmented_cloud.is_dense = True
        self.segmented_pub_.publish(segmented_cloud)
        self._publish_bounding_boxes(out_header=out_header, points_xyz=segmented_xyz)

        if self.log_interval_ns_ > 0 and (now_ns - self.last_info_ns_) >= self.log_interval_ns_:
            self.last_info_ns_ = now_ns
            self.get_logger().info(
                "Segmented cloud published: "
                f"{kept_points}/{total_points} points in ROI above table"
            )

    def _publish_bounding_boxes(self, out_header, points_xyz: Sequence[Tuple[float, float, float]]) -> None:
        if not self.publish_bounding_boxes_:
            return

        marker_array = MarkerArray()
        next_marker_id = 0

        if points_xyz:
            clusters = self._cluster_points(points_xyz)
            clusters.sort(key=len, reverse=True)
            for cluster in clusters:
                if len(cluster) < self.min_cluster_points_:
                    continue
                size_x, size_y, size_z, center_x, center_y, center_z = self._compute_bbox(
                    points_xyz, cluster
                )
                if not self._is_box_size_valid(size_x, size_y, size_z):
                    continue

                marker = Marker()
                marker.header = out_header
                marker.ns = "segmented_object_bbox"
                marker.id = next_marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = center_x
                marker.pose.position.y = center_y
                marker.pose.position.z = center_z
                marker.pose.orientation.w = 1.0
                marker.scale.x = size_x
                marker.scale.y = size_y
                marker.scale.z = size_z
                marker.color.a = 0.35
                marker.color.r = 0.2
                marker.color.g = 0.85
                marker.color.b = 0.2
                marker_array.markers.append(marker)
                next_marker_id += 1
                if next_marker_id >= self.max_bounding_boxes_:
                    break

        for old_id in range(next_marker_id, self.last_bbox_count_):
            delete_marker = Marker()
            delete_marker.header = out_header
            delete_marker.ns = "segmented_object_bbox"
            delete_marker.id = old_id
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        self.last_bbox_count_ = next_marker_id
        self.bbox_pub_.publish(marker_array)

    def _cluster_points(
        self, points_xyz: Sequence[Tuple[float, float, float]]
    ) -> List[List[int]]:
        if not points_xyz:
            return []

        cell_size = self.cluster_tolerance_m_
        sq_tolerance = self.cluster_tolerance_m_ * self.cluster_tolerance_m_

        cells: List[Tuple[int, int, int]] = []
        cell_to_indices: Dict[Tuple[int, int, int], List[int]] = defaultdict(list)

        for idx, (px, py, pz) in enumerate(points_xyz):
            cell = (
                int(math.floor(px / cell_size)),
                int(math.floor(py / cell_size)),
                int(math.floor(pz / cell_size)),
            )
            cells.append(cell)
            cell_to_indices[cell].append(idx)

        visited = [False] * len(points_xyz)
        clusters: List[List[int]] = []

        for start_idx in range(len(points_xyz)):
            if visited[start_idx]:
                continue

            queue = deque([start_idx])
            visited[start_idx] = True
            cluster: List[int] = []

            while queue:
                current_idx = queue.popleft()
                cluster.append(current_idx)
                current_point = points_xyz[current_idx]
                cx, cy, cz = cells[current_idx]

                for nx in range(cx - 1, cx + 2):
                    for ny in range(cy - 1, cy + 2):
                        for nz in range(cz - 1, cz + 2):
                            for neighbor_idx in cell_to_indices.get((nx, ny, nz), []):
                                if visited[neighbor_idx]:
                                    continue
                                neighbor_point = points_xyz[neighbor_idx]
                                dx = neighbor_point[0] - current_point[0]
                                dy = neighbor_point[1] - current_point[1]
                                dz = neighbor_point[2] - current_point[2]
                                if (dx * dx + dy * dy + dz * dz) > sq_tolerance:
                                    continue
                                visited[neighbor_idx] = True
                                queue.append(neighbor_idx)

            clusters.append(cluster)

        return clusters

    @staticmethod
    def _compute_bbox(
        points_xyz: Sequence[Tuple[float, float, float]],
        indices: Sequence[int],
    ) -> Tuple[float, float, float, float, float, float]:
        min_x = float("inf")
        max_x = float("-inf")
        min_y = float("inf")
        max_y = float("-inf")
        min_z = float("inf")
        max_z = float("-inf")

        for idx in indices:
            px, py, pz = points_xyz[idx]
            min_x = min(min_x, px)
            max_x = max(max_x, px)
            min_y = min(min_y, py)
            max_y = max(max_y, py)
            min_z = min(min_z, pz)
            max_z = max(max_z, pz)

        size_x = max(0.001, max_x - min_x)
        size_y = max(0.001, max_y - min_y)
        size_z = max(0.001, max_z - min_z)
        center_x = 0.5 * (min_x + max_x)
        center_y = 0.5 * (min_y + max_y)
        center_z = 0.5 * (min_z + max_z)
        return (size_x, size_y, size_z, center_x, center_y, center_z)

    def _is_box_size_valid(self, size_x: float, size_y: float, size_z: float) -> bool:
        if size_x < self.min_object_size_x_ or size_x > self.max_object_size_x_:
            return False
        if size_y < self.min_object_size_y_ or size_y > self.max_object_size_y_:
            return False
        if size_z < self.min_object_size_z_ or size_z > self.max_object_size_z_:
            return False
        return True

    def _warn_throttled(self, now_ns: int, message: str) -> None:
        if now_ns - self.last_warn_ns_ > 2_000_000_000:
            self.last_warn_ns_ = now_ns
            self.get_logger().warn(message)

    def _normalize_bounds(self, axis: str, lower: float, upper: float) -> Tuple[float, float]:
        if lower <= upper:
            return lower, upper
        self.get_logger().warn(
            f"Received {axis} bounds with min > max ({lower} > {upper}); swapping values."
        )
        return upper, lower

    def _is_inside_bounds(self, x: float, y: float, z: float) -> bool:
        if x < self.min_x_ or x > self.max_x_:
            return False
        if y < self.min_y_ or y > self.max_y_:
            return False
        if z < self.min_z_ or z > self.max_z_:
            return False
        return True

    @staticmethod
    def _transform_point(
        px: float,
        py: float,
        pz: float,
        tx: float,
        ty: float,
        tz: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> Tuple[float, float, float]:
        q_norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if q_norm > 1e-9:
            qx /= q_norm
            qy /= q_norm
            qz /= q_norm
            qw /= q_norm
        else:
            qx = 0.0
            qy = 0.0
            qz = 0.0
            qw = 1.0

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rx = (1.0 - 2.0 * (yy + zz)) * px + 2.0 * (xy - wz) * py + 2.0 * (xz + wy) * pz
        ry = 2.0 * (xy + wz) * px + (1.0 - 2.0 * (xx + zz)) * py + 2.0 * (yz - wx) * pz
        rz = 2.0 * (xz - wy) * px + 2.0 * (yz + wx) * py + (1.0 - 2.0 * (xx + yy)) * pz

        return (rx + tx, ry + ty, rz + tz)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectSegmentationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
