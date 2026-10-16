#!/usr/bin/env python3
"""Run the real large_demo mission; save odometry, sensor counts and acceptance results.

Run manually after building and sourcing the workspace. This is deliberately not
part of ordinary unit tests: it starts Gazebo and can take several minutes.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from patrol.health import cloud_quality, image_quality, imu_quality


class Observer(Node):
    def __init__(self):
        super().__init__('patrol_gazebo_acceptance')
        self.status = {}
        self.last_status_wall = None
        self.states = set()
        self.transitions = []
        self.poses = []
        self.plan = []
        self.commands = []
        self.sensor_counts = {'rgb': 0, 'depth': 0, 'lidar_points': 0, 'imu': 0}
        self.sensor_invalid = dict.fromkeys(self.sensor_counts, 0)
        self.sensor_invalid_examples = []
        self.survey_sensor_counts = dict.fromkeys(self.sensor_counts, 0)
        self.survey_min_fraction = {'depth': 1.0, 'lidar_points': 1.0}
        self.sensor_last_stamp = {}
        self.previous = None
        # Independent whole-cell check from measured odometry, for the default 25 m square.
        centres = np.arange(-12.25, 12.5, .5)
        xx, yy = np.meshgrid(centres, centres)
        self.centres = np.column_stack((xx.ravel(), yy.ravel()))
        self.covered = np.zeros(len(self.centres), dtype=bool)
        self.radius = 3.0-math.sqrt(2)*.25
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/parrot1/patrol/status', self.on_status, latched)
        self.create_subscription(RosPath, '/parrot1/patrol/planned_path', self.on_plan, latched)
        self.create_subscription(Odometry, '/parrot1/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/parrot1/cmd_vel', self.on_command, 10)
        self.create_subscription(Image, '/parrot1/camera/image', self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, '/parrot1/camera/depth/image', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/parrot1/scan/points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/parrot1/imu', self.on_imu, qos_profile_sensor_data)
        self.pause_client = self.create_client(SetBool, '/parrot1/patrol/pause')

    def on_status(self, msg):
        self.last_status_wall = time.monotonic()
        self.status = json.loads(msg.data)
        state = self.status['state']
        self.states.add(state)
        if not self.transitions or self.transitions[-1]['state'] != state:
            self.transitions.append({'state': state, 'wall_time': time.monotonic(),
                                     'position': self.status.get('position')})

    def on_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]

    def on_rgb(self, msg):
        self.record_sensor('rgb', msg, image_quality(msg))

    def on_depth(self, msg):
        self.record_sensor('depth', msg, image_quality(msg, depth=True))

    def on_cloud(self, msg):
        self.record_sensor('lidar_points', msg, cloud_quality(msg))

    def on_imu(self, msg):
        self.record_sensor('imu', msg, imu_quality(msg))

    def record_sensor(self, name, msg, result):
        valid, reason, metrics = result
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp <= self.sensor_last_stamp.get(name, -1):
            valid = False
            reason = 'Timestamp did not advance'
        self.sensor_last_stamp[name] = stamp
        if not valid:
            self.sensor_invalid[name] += 1
            if len(self.sensor_invalid_examples) < 10:
                self.sensor_invalid_examples.append({'sensor': name, 'stamp': stamp, 'reason': reason,
                                                     'metrics': metrics, 'state': self.status.get('state'),
                                                     'pose': self.poses[-1][1:4] if self.poses else None})
            return
        self.sensor_counts[name] += 1
        if self.status.get('state') == 'PATROL':
            self.survey_sensor_counts[name] += 1
            if name in self.survey_min_fraction:
                self.survey_min_fraction[name] = min(self.survey_min_fraction[name], metrics['valid_fraction'])

    def on_command(self, msg):
        self.commands.append((msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        state = self.status.get('state', 'WAITING')
        self.poses.append((stamp, p.x, p.y, p.z, state))
        if state != 'PATROL' or abs(p.z-15.0) > .2:
            self.previous = None
            return
        end = np.array([p.x, p.y])
        start = end
        if self.previous is not None:
            old_stamp, old_xy = self.previous
            dt = stamp-old_stamp
            if 0 < dt <= 1.0 and np.linalg.norm(end-old_xy) <= 3.0*dt+.5:
                start = old_xy
        delta = end-start
        length_sq = float(delta@delta)
        if length_sq:
            fraction = np.clip((self.centres-start)@delta/length_sq, 0., 1.)
            nearest = start+fraction[:,None]*delta
        else:
            nearest = start
        self.covered |= np.linalg.norm(self.centres-nearest, axis=1) <= self.radius+1e-9
        self.previous = (stamp, end)

    def pause(self, enabled):
        if not self.pause_client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Patrol pause service unavailable')
        future = self.pause_client.call_async(SetBool.Request(data=enabled))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if not future.done() or not future.result().success:
            raise RuntimeError('Pause/resume request failed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('patrol_validation'))
    parser.add_argument('--timeout', type=float, default=1200)
    parser.add_argument('--mode', choices=('standalone', 'attach'), default='standalone')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # A separate DDS domain and Gazebo transport partition avoid other simulations.
    os.environ.setdefault('ROS_DOMAIN_ID', '70')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['IGN_PARTITION'] = 'patrol_validation_' + str(os.getpid())
    os.environ['GZ_PARTITION'] = os.environ['IGN_PARTITION']
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['ROS_LOG_DIR'] = str(args.output.resolve()/'ros_logs')
    rclpy.init()
    node = Observer()
    result = {'success': False, 'mode': args.mode, 'world': 'large_demo', 'area_m': [25,25],
              'target_altitude_m': 15., 'footprint_diameter_m': 6.,
              'coverage_model': 'Geometric circular footprint; no occlusion or image-quality guarantee.'}
    log = (args.output/'gazebo_launch.log').open('w')
    process = None
    simulation = None
    start = time.monotonic()
    next_report = start
    paused_at = None
    pause_pose = None
    pause_done = False
    complete_at = None
    try:
        if args.mode == 'attach':
            simulation = subprocess.Popen(
                ['ros2', 'launch', '41068_ignition_bringup', '41068_ignition.launch.py',
                 'world:=large_demo', 'gz_gui:=false', 'husky:=false', 'parrot:=true',
                 'slam:=false', 'nav2:=false', 'rviz:=false',
                 'parrot_x:=0.0', 'parrot_y:=-8.0', 'parrot_z:=0.8',
                 'parrot_camera_pitch:=1.5707963267948966'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        process = subprocess.Popen(
            ['ros2','launch','41068_ignition_bringup','41068_coverage_patrol.launch.py',
             'world:=large_demo','gz_gui:=false', 'rendering:=software', 'mode:=' + args.mode],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        while time.monotonic()-start < args.timeout:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            state = node.status.get('state')
            if now >= next_report:
                print(json.dumps({'elapsed_s': round(now-start), 'state': state,
                                  'waypoint': node.status.get('waypoint_index'),
                                  'coverage': node.status.get('coverage_fraction'),
                                  'measured_coverage': float(node.covered.mean()),
                                  'position': node.poses[-1][1:4] if node.poses else None,
                                  'sensors': node.sensor_counts}), flush=True)
                next_report = now+20
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited with code {process.returncode}; see gazebo_launch.log')
            if simulation is not None and simulation.poll() is not None:
                raise RuntimeError('Base simulation exited during attach-mode acceptance')
            if now - (node.last_status_wall or start) > 20:
                raise RuntimeError('Patrol status heartbeat missing for 20 s; inspect node exit / launch log')
            if state in ('ERROR','STOPPED'):
                raise RuntimeError('Patrol failed: '+node.status.get('reason',''))
            if now-start > 45 and not node.poses:
                raise RuntimeError('No Gazebo odometry received within 45 seconds')
            if node.count_publishers('/parrot1/cmd_vel') > 1:
                raise RuntimeError('Multiple flight controllers publish cmd_vel')
            if not pause_done and paused_at is None and state == 'PATROL' and node.status.get('waypoint_index',0) >= 3:
                node.pause(True)
                paused_at = time.monotonic()
            if paused_at is not None and not pause_done and state == 'PAUSED':
                if pause_pose is None and now-paused_at > 1.5:
                    pause_pose = node.poses[-1][1:4]
                if pause_pose is not None and now-paused_at > 3:
                    result['pause_drift_m'] = math.dist(node.poses[-1][1:4], pause_pose)
                    assert result['pause_drift_m'] < .1, 'Drone drifted while paused'
                    node.pause(False)
                    pause_done = True
            if state == 'COMPLETE':
                if complete_at is None:
                    complete_at = now
                if now-complete_at < 3:
                    continue
                final = node.poses[-1][1:4]
                home = node.poses[0][1:3]
                result.update(
                    elapsed_wall_seconds=round(now-start,2),
                    elapsed_sim_seconds=round(node.poses[-1][0]-node.poses[0][0],2),
                    initial_position=list(node.poses[0][1:4]), final_position=list(final),
                    home_error_m=math.dist(final[:2],home),
                    altitude_error_m=abs(final[2]-15.),
                    measured_covered_cells=int(node.covered.sum()), total_cells=len(node.covered),
                    measured_coverage_fraction=float(node.covered.mean()),
                    planned_waypoints=len(node.plan), pause_resume_verified=pause_done,
                    sensor_messages=node.sensor_counts,
                    max_observed_altitude_m=max(p[3] for p in node.poses))
                assert {'TAKEOFF','PATROL','RETURNING','COMPLETE'} <= node.states, 'Missing mission phase'
                assert node.status['completed_passes'] == 1
                assert node.status['coverage_fraction'] == 1.
                assert node.covered.all(), 'Measured flight left uncovered grid cells'
                assert result['home_error_m'] <= .15, 'Did not return to start XY'
                assert result['altitude_error_m'] <= .2, 'Return height incorrect'
                assert len(node.plan) == 119, 'Unexpected default coverage route'
                assert pause_done, 'Pause/resume was not exercised'
                assert node.sensor_counts['rgb'] > 0, 'No camera images'
                assert node.sensor_counts['lidar_points'] > 0, 'No lidar point clouds'
                assert all(count > 0 for count in node.survey_sensor_counts.values()), 'Missing valid survey sensor data'
                assert not any(node.sensor_invalid.values()), 'Invalid or frozen sensor data received'
                assert node.status.get('flight_ready'), 'Mission completed without healthy required streams'
                assert max(p[3] for p in node.poses) < 15.5, 'Unexpected altitude overshoot'
                result['success'] = True
                break
        else:
            raise TimeoutError('Full mission exceeded wall-clock time limit')
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        for child in (process, simulation):
            if child is not None and child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=5)
        result.update(final_status=node.status, states=sorted(node.states),
                      odometry_samples=len(node.poses), sensor_messages=node.sensor_counts,
                      sensor_invalid_messages=node.sensor_invalid,
                      sensor_invalid_examples=node.sensor_invalid_examples,
                      survey_sensor_messages=node.survey_sensor_counts,
                      survey_min_valid_fraction=node.survey_min_fraction,
                      transitions=[dict(t, wall_time=round(t['wall_time']-start,2)) for t in node.transitions])
        with (args.output/'trajectory.csv').open('w',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['sim_time','x','y','z','state'])
            writer.writerows(node.poses)
        (args.output/'planned_path.json').write_text(json.dumps(node.plan))
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
