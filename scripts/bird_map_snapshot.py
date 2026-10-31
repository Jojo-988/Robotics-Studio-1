#!/usr/bin/env python3
"""Capture an exact-stamp map/statistics pair without controlling the robot."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml

from map_data.snapshot_pairing import SnapshotPairs, stamp_ns


def plain(message):
    return json.loads(json.dumps(message_to_ordereddict(message)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='New directory; an existing directory is never overwritten')
    parser.add_argument('--map-topic', default='/parrot1/bird_map')
    parser.add_argument('--statistics-topic', default='/parrot1/bird_map_statistics')
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    node = Node('bird_map_snapshot')
    pairs = SnapshotPairs(capacity=64)
    qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, args.map_topic,
                             lambda msg: pairs.add_map(plain(msg)), qos)
    node.create_subscription(String, args.statistics_topic,
                             lambda msg: pairs.add_statistics(json.loads(msg.data)), qos)
    started = time.monotonic()
    try:
        grid, statistics, counts = pairs.wait(
            lambda dt: rclpy.spin_once(node, timeout_sec=dt), args.timeout)
        (args.output/'map.yaml').write_text(yaml.safe_dump(grid, sort_keys=False), encoding='utf-8')
        (args.output/'statistics.json').write_text(json.dumps(statistics, indent=2), encoding='utf-8')
        report = {'pass': True, 'source_stamp_ns': stamp_ns(grid['header']['stamp']),
                  'frame_id': grid['header']['frame_id'], 'counts': counts,
                  'map_topic': args.map_topic, 'statistics_topic': args.statistics_topic,
                  'wait_wall_seconds': time.monotonic()-started,
                  'limitation': 'Retained snapshot allowed; does not prove live updates or independent geometric accuracy. No odometry pairing.'}
        (args.output/'validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        print('Snapshot saved to ' + str(args.output.resolve()))
        return 0
    except Exception as error:
        report = {'pass': False, 'error': str(error), 'error_type': type(error).__name__}
        (args.output/'failure.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
