"""``vision_node``: Image → ``fret.vision`` → PointStamped ball (FR-HW-31)."""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
import numpy.typing as npt

from fret.vision.types import BallObservation, CameraFrame


class _Pipeline(Protocol):
    def process(self, frames: list[CameraFrame]) -> BallObservation | None: ...


class VisionNodeLogic:
    """ROS-free core used by tests and the ROS shell."""

    def __init__(
        self,
        pipeline: _Pipeline,
        *,
        rate_hz: float = 5.0,
        intrinsics_id: str = "cam0",
    ) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self._pipeline = pipeline
        self.rate_hz = rate_hz
        self._intrinsics_id = intrinsics_id
        self.last_observation: dict[str, Any] | None = None

    def process_bgr(
        self,
        image_bgr: npt.NDArray[np.uint8],
        *,
        stamp_s: float,
        frame_id: str = "cam0",
    ) -> dict[str, Any] | None:
        # Pipeline contract uses RGB; convert BGR → RGB.
        image_rgb = image_bgr[:, :, ::-1].copy()
        frame = CameraFrame(
            camera_id=frame_id,
            image=image_rgb,
            timestamp=stamp_s,
            intrinsics_id=self._intrinsics_id,
        )
        obs = self._pipeline.process([frame])
        if obs is None:
            self.last_observation = None
            return None
        payload = {
            "frame_id": "world",
            "x": float(obs.position_world[0]),
            "y": float(obs.position_world[1]),
            "z": float(obs.position_world[2]),
            "confidence": 1.0 if obs.pickable is not False else 0.0,
            "stamp_s": stamp_s,
        }
        self.last_observation = payload
        return payload


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    """ROS entry point for the ``vision_node`` console script."""
    import argparse

    import rclpy
    from geometry_msgs.msg import PointStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    from fret.vision.factory import build_hsv_plane_pipeline

    parser = argparse.ArgumentParser(description="FRET vision_node")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--image-topic", default="/fret/camera/image_raw")
    parser.add_argument("--point-topic", default="/fret/vision/ball")
    parser.add_argument("--vision-config", default=None)
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node: Any = Node("vision_node")
    pipeline = build_hsv_plane_pipeline(parsed.vision_config)
    logic = VisionNodeLogic(pipeline, rate_hz=parsed.rate)
    pub = node.create_publisher(PointStamped, parsed.point_topic, 10)

    def _on_image(msg: Image) -> None:
        h, w = int(msg.height), int(msg.width)
        if msg.encoding not in {"bgr8", "rgb8"}:
            node.get_logger().warning(f"unsupported encoding {msg.encoding}")
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            image = arr.reshape((h, w, 3))
        except ValueError:
            return
        if msg.encoding == "rgb8":
            # process_bgr expects BGR
            image = image[:, :, ::-1].copy()
        stamp_s = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        obs = logic.process_bgr(
            image, stamp_s=stamp_s, frame_id=msg.header.frame_id or "cam0"
        )
        if obs is None:
            return
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "world"
        out.point.x = obs["x"]
        out.point.y = obs["y"]
        out.point.z = obs["z"]
        pub.publish(out)

    node.create_subscription(Image, parsed.image_topic, _on_image, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if bool(getattr(rclpy, "ok")()):
            rclpy.shutdown()
