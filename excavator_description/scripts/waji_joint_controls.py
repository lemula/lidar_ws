#!/usr/bin/python3
import math
from typing import Dict

import rclpy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


JOINTS = [
    {"name": "swing_joint", "min": -1.57, "max": 1.57, "default": 0.0, "y": 0.90},
    {"name": "boom_joint", "min": -3.14, "max": 3.14, "default": 0.0, "y": 0.60},
    {"name": "arm_joint", "min": -3.14, "max": 3.14, "default": 0.0, "y": 0.30},
    {"name": "bucket_joint", "min": -3.14, "max": 3.14, "default": 0.0, "y": 0.00},
]

SLIDER_MIN_X = -3.14
SLIDER_MAX_X = 3.14
SLIDER_Z = 0.80


class WajiJointControls(Node):
    def __init__(self):
        super().__init__("waji_joint_controls")

        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 30.0).value)

        self.joint_values: Dict[str, float] = {}
        for joint in JOINTS:
            name = joint["name"]
            value = float(self.declare_parameter(name, joint["default"]).value)
            self.joint_values[name] = self._clamp(value, joint["min"], joint["max"])

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.server = InteractiveMarkerServer(self, "waji_joint_controls")

        for joint in JOINTS:
            self._insert_slider(joint)
        self.server.applyChanges()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period, self.publish_joint_state)
        self.publish_joint_state()

        self.get_logger().info(
            "RViz joint controls ready: Add InteractiveMarkers topic "
            "/waji_joint_controls/update if it is not visible."
        )

    def _insert_slider(self, joint):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = f"{joint['name']}_slider"
        marker.description = self._description(joint)
        marker.scale = 0.22
        marker.pose.position.x = self._angle_to_x(self.joint_values[joint["name"]], joint)
        marker.pose.position.y = joint["y"]
        marker.pose.position.z = SLIDER_Z
        marker.pose.orientation.w = 1.0

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.interaction_mode = InteractiveMarkerControl.NONE
        visual.markers.append(self._handle_marker(joint["name"]))
        visual.markers.append(self._text_marker(joint))
        marker.controls.append(visual)

        move = InteractiveMarkerControl()
        move.name = f"{joint['name']}_move_x"
        move.orientation.w = 1.0
        move.orientation_mode = InteractiveMarkerControl.FIXED
        move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move.description = f"Drag to set {joint['name']}"
        marker.controls.append(move)

        self.server.insert(marker, feedback_callback=lambda feedback, j=joint: self._on_feedback(j, feedback))

    def _handle_marker(self, joint_name: str) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color = ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0)
        marker.ns = joint_name
        return marker

    def _text_marker(self, joint) -> Marker:
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 0.16
        marker.scale.z = 0.10
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = self._description(joint)
        marker.ns = f"{joint['name']}_label"
        return marker

    def _on_feedback(self, joint, feedback):
        if feedback.event_type != feedback.POSE_UPDATE:
            return

        x = self._clamp(feedback.pose.position.x, SLIDER_MIN_X, SLIDER_MAX_X)
        self.joint_values[joint["name"]] = self._x_to_angle(x, joint)
        self._insert_slider(joint)
        self.server.applyChanges()
        self.publish_joint_state()

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint["name"] for joint in JOINTS]
        msg.position = [self.joint_values[joint["name"]] for joint in JOINTS]
        self.joint_pub.publish(msg)

    def _description(self, joint) -> str:
        value = self.joint_values[joint["name"]]
        return f"{joint['name']}: {value:.3f} rad / {math.degrees(value):.1f} deg"

    def _angle_to_x(self, value: float, joint) -> float:
        ratio = (value - joint["min"]) / (joint["max"] - joint["min"])
        return SLIDER_MIN_X + ratio * (SLIDER_MAX_X - SLIDER_MIN_X)

    def _x_to_angle(self, x: float, joint) -> float:
        ratio = (x - SLIDER_MIN_X) / (SLIDER_MAX_X - SLIDER_MIN_X)
        return joint["min"] + ratio * (joint["max"] - joint["min"])

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def destroy_node(self):
        try:
            self.server.shutdown()
            super().destroy_node()
        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    node = WajiJointControls()
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
