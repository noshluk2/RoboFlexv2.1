#!/usr/bin/env python3

from typing import Optional, Tuple

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


class SceneIntegrationNode(Node):
    def __init__(self) -> None:
        super().__init__("scene_integration_node")

        self.declare_parameter("pointcloud_topic", "/camera/depth/color/points")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("object_id", "camera_segmented_object")
        self.declare_parameter("sample_stride", 8)
        self.declare_parameter("min_points_for_object", 120)
        self.declare_parameter("update_period_sec", 0.4)
        self.declare_parameter("box_padding_m", 0.015)
        self.declare_parameter("workspace_min_x", 0.05)
        self.declare_parameter("workspace_max_x", 0.45)
        self.declare_parameter("workspace_min_y", -0.30)
        self.declare_parameter("workspace_max_y", 0.30)
        self.declare_parameter("workspace_min_z", -0.05)
        self.declare_parameter("workspace_max_z", 0.40)

        self.pointcloud_topic_ = self.get_parameter("pointcloud_topic").value
        self.base_frame_ = self.get_parameter("base_frame").value
        self.object_id_ = self.get_parameter("object_id").value
        self.sample_stride_ = max(1, int(self.get_parameter("sample_stride").value))
        self.min_points_for_object_ = int(self.get_parameter("min_points_for_object").value)
        self.update_period_ns_ = int(float(self.get_parameter("update_period_sec").value) * 1e9)
        self.box_padding_m_ = max(0.0, float(self.get_parameter("box_padding_m").value))
        self.workspace_min_x_ = float(self.get_parameter("workspace_min_x").value)
        self.workspace_max_x_ = float(self.get_parameter("workspace_max_x").value)
        self.workspace_min_y_ = float(self.get_parameter("workspace_min_y").value)
        self.workspace_max_y_ = float(self.get_parameter("workspace_max_y").value)
        self.workspace_min_z_ = float(self.get_parameter("workspace_min_z").value)
        self.workspace_max_z_ = float(self.get_parameter("workspace_max_z").value)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.apply_scene_client_ = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        self.last_update_ns_ = 0
        self.last_warn_ns_ = 0
        self.object_present_ = False

        self.pointcloud_sub_ = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic_,
            self._pointcloud_callback,
            5,
        )

        self.get_logger().info(
            f"Scene integration started. pointcloud={self.pointcloud_topic_} "
            f"base_frame={self.base_frame_}"
        )

    def _pointcloud_callback(self, cloud_msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.update_period_ns_ > 0 and (now_ns - self.last_update_ns_) < self.update_period_ns_:
            return
        self.last_update_ns_ = now_ns

        if not self.apply_scene_client_.service_is_ready():
            if now_ns - self.last_warn_ns_ > 2_000_000_000:
                self.last_warn_ns_ = now_ns
                self.get_logger().warn("/apply_planning_scene service is not ready yet")
            return

        try:
            transform = self.tf_buffer_.lookup_transform(
                self.base_frame_,
                cloud_msg.header.frame_id,
                Time(),
            )
        except TransformException as exc:
            if now_ns - self.last_warn_ns_ > 2_000_000_000:
                self.last_warn_ns_ = now_ns
                self.get_logger().warn(
                    f"Missing TF from {cloud_msg.header.frame_id} to "
                    f"{self.base_frame_}: {exc}"
                )
            return

        bounds = self._extract_workspace_bounds(cloud_msg, transform)
        if bounds is None:
            if self.object_present_:
                self._publish_remove_object()
                self.object_present_ = False
            return

        min_x, max_x, min_y, max_y, min_z, max_z, point_count = bounds
        self._publish_box_object(min_x, max_x, min_y, max_y, min_z, max_z, point_count)
        self.object_present_ = True

    def _extract_workspace_bounds(self, cloud_msg: PointCloud2, transform) -> Optional[Tuple[float, float, float, float, float, float, int]]:
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        points_iter = point_cloud2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        min_x = float("inf")
        min_y = float("inf")
        min_z = float("inf")
        max_x = float("-inf")
        max_y = float("-inf")
        max_z = float("-inf")
        accepted = 0

        for idx, point in enumerate(points_iter):
            if self.sample_stride_ > 1 and (idx % self.sample_stride_) != 0:
                continue

            px, py, pz = float(point[0]), float(point[1]), float(point[2])
            bx, by, bz = self._transform_point(px, py, pz, tx, ty, tz, qx, qy, qz, qw)
            if not self._is_inside_workspace(bx, by, bz):
                continue

            min_x = min(min_x, bx)
            min_y = min(min_y, by)
            min_z = min(min_z, bz)
            max_x = max(max_x, bx)
            max_y = max(max_y, by)
            max_z = max(max_z, bz)
            accepted += 1

        if accepted < self.min_points_for_object_:
            return None

        return (min_x, max_x, min_y, max_y, min_z, max_z, accepted)

    def _is_inside_workspace(self, x: float, y: float, z: float) -> bool:
        if x < self.workspace_min_x_ or x > self.workspace_max_x_:
            return False
        if y < self.workspace_min_y_ or y > self.workspace_max_y_:
            return False
        if z < self.workspace_min_z_ or z > self.workspace_max_z_:
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

    def _publish_box_object(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        min_z: float,
        max_z: float,
        point_count: int,
    ) -> None:
        size_x = max(0.01, (max_x - min_x) + self.box_padding_m_)
        size_y = max(0.01, (max_y - min_y) + self.box_padding_m_)
        size_z = max(0.01, (max_z - min_z) + self.box_padding_m_)

        center_x = (min_x + max_x) * 0.5
        center_y = (min_y + max_y) * 0.5
        center_z = (min_z + max_z) * 0.5

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]

        pose = Pose()
        pose.position.x = center_x
        pose.position.y = center_y
        pose.position.z = center_z
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = self.base_frame_
        collision.id = self.object_id_
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD

        self._publish_planning_scene_diff(collision)
        self.get_logger().debug(
            f"Updated object {self.object_id_} from {point_count} points "
            f"(size={size_x:.3f} {size_y:.3f} {size_z:.3f})"
        )

    def _publish_remove_object(self) -> None:
        collision = CollisionObject()
        collision.header.frame_id = self.base_frame_
        collision.id = self.object_id_
        collision.operation = CollisionObject.REMOVE
        self._publish_planning_scene_diff(collision)
        self.get_logger().info(f"Removed object {self.object_id_} (insufficient points)")

    def _publish_planning_scene_diff(self, collision: CollisionObject) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.apply_scene_client_.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneIntegrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
