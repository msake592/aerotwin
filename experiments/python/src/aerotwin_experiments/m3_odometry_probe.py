"""Internal live probe, using the ROS underlay only when explicitly invoked."""

import argparse
import json
import math
import time


def odometry_sample(message) -> dict:
    position = list(message.position)
    velocity = list(message.velocity)
    attitude = list(message.q)
    valid = (
        message.pose_frame in (1, 2)
        and message.velocity_frame in (1, 2, 3)
        and all(math.isfinite(value) for value in position + velocity + attitude)
        and math.isclose(math.hypot(*attitude), 1.0, abs_tol=1e-3)
        and 0 < message.timestamp_sample <= message.timestamp
    )
    return {
        "timestamp_us": message.timestamp,
        "timestamp_sample_us": message.timestamp_sample,
        "pose_frame": message.pose_frame,
        "velocity_frame": message.velocity_frame,
        "reset_counter": message.reset_counter,
        "position_m": position if all(map(math.isfinite, position)) else None,
        "velocity_mps": velocity if all(map(math.isfinite, velocity)) else None,
        "attitude_wxyz": attitude if all(map(math.isfinite, attitude)) else None,
        "valid": valid,
    }


def sample_series_valid(samples: list[dict]) -> bool:
    return (
        len(samples) >= 3
        and all(s["valid"] for s in samples)
        and len({s["reset_counter"] for s in samples}) == 1
        and all(
            first["timestamp_us"] < second["timestamp_us"]
            and first["timestamp_sample_us"] < second["timestamp_sample_us"]
            for first, second in zip(samples, samples[1:])
        )
    )


def qos_allows_observation(compatibility: str) -> bool:
    return compatibility in {"OK", "WARNING"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic_prefix")
    args = parser.parse_args()
    details = {
        "discovered": False,
        "subscription_created": False,
        "received": 0,
        "decoded": 0,
        "semantics_passed": False,
    }
    try:
        import rclpy
        from px4_msgs.msg import VehicleOdometry
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
            qos_check_compatible,
        )

        rclpy.init(args=[])
        try:
            node = rclpy.create_node("aerotwin_m32_odometry")
            try:
                deadline = time.monotonic() + 10
                candidates = []
                type_name = "px4_msgs/msg/VehicleOdometry"
                while time.monotonic() < deadline:
                    candidates = [
                        name
                        for name, types in node.get_topic_names_and_types()
                        if name in {args.topic_prefix, args.topic_prefix + "_v0"}
                        and types == [type_name]
                    ]
                    if candidates:
                        break
                    rclpy.spin_once(node, timeout_sec=0.1)
                if len(candidates) != 1:
                    raise RuntimeError(f"Expected one typed PX4 odometry topic, found {candidates}")
                topic = candidates[0]
                details.update(discovered=True, topic=topic, type=type_name)
                endpoints = node.get_publishers_info_by_topic(topic)
                if len(endpoints) != 1:
                    raise RuntimeError(f"Expected one odometry publisher, found {len(endpoints)}")
                offered = endpoints[0].qos_profile
                requested = QoSProfile(
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=5,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                )
                details["offered_qos"] = {
                    "reliability": offered.reliability.name,
                    "durability": offered.durability.name,
                    "history": offered.history.name,
                    "depth": offered.depth,
                }
                details["requested_qos"] = {
                    "reliability": requested.reliability.name,
                    "durability": requested.durability.name,
                    "history": requested.history.name,
                    "depth": requested.depth,
                }
                compatibility, reason = qos_check_compatible(offered, requested)
                details["qos_compatibility"] = compatibility.name
                if not qos_allows_observation(compatibility.name):
                    raise RuntimeError(f"Incompatible publisher QoS: {reason}")
                samples = []

                def received(message):
                    details["received"] += 1
                    samples.append(odometry_sample(message))
                    details["decoded"] += 1

                node.create_subscription(VehicleOdometry, topic, received, requested)
                details["subscription_created"] = True
                deadline = time.monotonic() + 8
                while len(samples) < 3 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                details["samples"] = samples
                final_endpoints = node.get_publishers_info_by_topic(topic)
                details["publisher_gid"] = list(endpoints[0].endpoint_gid)
                details["publisher_stable"] = (
                    len(final_endpoints) == 1
                    and final_endpoints[0].endpoint_gid == endpoints[0].endpoint_gid
                )
                details["semantics_passed"] = details["publisher_stable"] and sample_series_valid(
                    samples
                )
            finally:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
    except (ImportError, RuntimeError, OSError) as error:
        details["error"] = str(error)
    print(json.dumps(details))
    return 0 if details["semantics_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
