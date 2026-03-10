#!/usr/bin/env python3
"""Benchmark CDR deserialization and reserialization from MCAP — Python (pycdr2) timing.

Reads all messages from MCAP files in testdata/mcap/ and measures
per-schema-type deser and reser time using pycdr2.

Usage: python scripts/bench_mcap_cdr.py
"""

import time
from collections import defaultdict
from pathlib import Path

from mcap.reader import make_reader

from edgefirst.schemas import (
    builtin_interfaces,
    edgefirst_msgs,
    foxglove_msgs,
    geometry_msgs,
    sensor_msgs,
    std_msgs,
)

TESTDATA_DIR = Path(__file__).parent.parent / "testdata" / "mcap"

# Map schema names to pycdr2 dataclass types for deserialization
SCHEMA_MAP = {
    "sensor_msgs/msg/Image": sensor_msgs.Image,
    "sensor_msgs/msg/CompressedImage": sensor_msgs.CompressedImage,
    "sensor_msgs/msg/Imu": sensor_msgs.Imu,
    "sensor_msgs/msg/NavSatFix": sensor_msgs.NavSatFix,
    "sensor_msgs/msg/PointCloud2": sensor_msgs.PointCloud2,
    "sensor_msgs/msg/CameraInfo": sensor_msgs.CameraInfo,
    "geometry_msgs/msg/TransformStamped": geometry_msgs.TransformStamped,
    "geometry_msgs/msg/TwistStamped": geometry_msgs.TwistStamped,
    "geometry_msgs/msg/AccelStamped": geometry_msgs.AccelStamped,
    "geometry_msgs/msg/Pose": geometry_msgs.Pose,
    "geometry_msgs/msg/Transform": geometry_msgs.Transform,
    "geometry_msgs/msg/Vector3": geometry_msgs.Vector3,
    "geometry_msgs/msg/Quaternion": geometry_msgs.Quaternion,
    "geometry_msgs/msg/Point": geometry_msgs.Point,
    "geometry_msgs/msg/Twist": geometry_msgs.Twist,
    "foxglove_msgs/msg/CompressedVideo": foxglove_msgs.CompressedVideo,
    "edgefirst_msgs/msg/Detect": edgefirst_msgs.Detect,
    "edgefirst_msgs/msg/DmaBuffer": edgefirst_msgs.DmaBuffer,
    "edgefirst_msgs/msg/Mask": edgefirst_msgs.Mask,
    "edgefirst_msgs/msg/ModelInfo": edgefirst_msgs.ModelInfo,
    "edgefirst_msgs/msg/RadarCube": edgefirst_msgs.RadarCube,
    "edgefirst_msgs/msg/RadarInfo": edgefirst_msgs.RadarInfo,
    "edgefirst_msgs/msg/Box": edgefirst_msgs.Box,
    "edgefirst_msgs/msg/Track": edgefirst_msgs.Track,
    "edgefirst_msgs/msg/Model": edgefirst_msgs.Model,
    "edgefirst_msgs/msg/LocalTime": edgefirst_msgs.LocalTime,
}


def bench_mcap(mcap_path: Path):
    """Benchmark CDR deser and reser for all messages in an MCAP file."""
    print(f"\n{'='*80}")
    print(f"Python CDR timing: {mcap_path.name}")
    print(f"{'='*80}")

    # Collect raw message data grouped by schema
    schema_messages: dict[str, list[bytes]] = defaultdict(list)
    total_bytes = 0

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, _channel, message in reader.iter_messages():
            name = schema.name if schema else "unknown"
            schema_messages[name].append(message.data)
            total_bytes += len(message.data)

    total_msgs = sum(len(msgs) for msgs in schema_messages.values())
    print(f"Loaded {total_msgs} messages ({total_bytes / 1024 / 1024:.1f} MB)\n")

    print(f"{'Schema':<40} {'Count':>6} {'Deser ms':>10} {'Reser ms':>10} {'Per msg':>10} {'MB/s':>10}")
    print(f"{'-'*40} {'-'*6} {'-'*10} {'-'*10} {'-'*10} {'-'*10}")

    grand_deser_us = 0.0
    grand_reser_us = 0.0

    for schema_name in sorted(schema_messages.keys()):
        messages = schema_messages[schema_name]
        cls = SCHEMA_MAP.get(schema_name)
        if cls is None:
            print(f"{schema_name:<40} {'SKIP':>6}")
            continue

        schema_bytes = sum(len(m) for m in messages)

        # Warm up
        for msg_data in messages[:min(3, len(messages))]:
            obj = cls.deserialize(msg_data)
            cls.serialize(obj)

        # Timed deserialization
        t0 = time.perf_counter_ns()
        deserialized = []
        for msg_data in messages:
            deserialized.append(cls.deserialize(msg_data))
        deser_ns = time.perf_counter_ns() - t0

        # Timed reserialization
        t0 = time.perf_counter_ns()
        for obj in deserialized:
            cls.serialize(obj)
        reser_ns = time.perf_counter_ns() - t0

        deser_ms = deser_ns / 1_000_000
        reser_ms = reser_ns / 1_000_000
        total_ns = deser_ns + reser_ns
        per_msg_us = total_ns / len(messages) / 1_000
        throughput = (schema_bytes / (total_ns / 1_000_000_000)) / (1024 * 1024) if total_ns > 0 else 0
        grand_deser_us += deser_ns / 1_000
        grand_reser_us += reser_ns / 1_000

        print(f"{schema_name:<40} {len(messages):>6} {deser_ms:>9.2f}ms {reser_ms:>9.2f}ms {per_msg_us:>8.1f}us {throughput:>9.1f}")

    print(f"\n{'TOTAL':<40} {total_msgs:>6} {grand_deser_us/1000:>9.2f}ms {grand_reser_us/1000:>9.2f}ms")


def main():
    mcap_files = sorted(TESTDATA_DIR.glob("*.mcap"))
    if not mcap_files:
        print("No MCAP files found in testdata/mcap/")
        return

    for mcap_path in mcap_files:
        bench_mcap(mcap_path)


if __name__ == "__main__":
    main()
