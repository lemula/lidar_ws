import rclpy
from excavator_interfaces.msg import DigTarget
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class DigPointScaffold(Node):
    """Validate height-map and bucket TF connectivity without generating a target."""

    def __init__(self) -> None:
        super().__init__("dig_point_scaffold")
        self.world_frame = str(
            self.declare_parameter("world_frame", "world").value
        )
        self.bucket_tip_frame = str(
            self.declare_parameter("bucket_tip_frame", "bucket_tip").value
        )
        self.enable_selection = bool(
            self.declare_parameter("enable_selection", False).value
        )
        self.declare_parameter("maximum_map_age_sec", 0.50)
        self.declare_parameter("minimum_confidence", 0.70)
        self.declare_parameter("minimum_reach_m", 1.0)
        self.declare_parameter("maximum_reach_m", 10.0)
        self.declare_parameter("edge_margin_m", 0.50)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(DigTarget, "dig_target", 5)
        self.subscription = self.create_subscription(
            GridMap, "grid_map", self._on_map, 1
        )
        self.get_logger().warning(
            "dig-point algorithm pending; only invalid status messages are published"
        )

    def _on_map(self, message: GridMap) -> None:
        target = DigTarget()
        target.header = message.header
        target.valid = False
        target.confidence = 0.0
        target.status = "selector_disabled"
        try:
            self.tf_buffer.lookup_transform(
                self.world_frame,
                self.bucket_tip_frame,
                Time.from_msg(message.header.stamp),
            )
        except TransformException as error:
            target.status = f"bucket_tf_unavailable: {error}"
        else:
            target.status = (
                "selector_not_implemented"
                if self.enable_selection
                else "inputs_connected_selector_disabled"
            )
        self.publisher.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DigPointScaffold()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

