
import json
import math
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SyntheticRover(Node):
    def __init__(self):
        super().__init__('synthetic_rover', namespace='husky1', cli_args=[
            '--ros-args', '-r', '/tf:=/husky1/tf', '-r', '/tf_static:=/husky1/tf_static'])
        self.x, self.y, self.yaw = -2.0, 0.0, 0.0
        self.v, self.w = 0.0, 0.0
        self.last_command = 0.0
        self.last_tick = time.monotonic()
        self.status = {}
        self.paths = []
        self.collisions = 0
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'rover/goal', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.state_sub = self.create_subscription(String, 'rover/status', self.state, 10)
        self.path_sub = self.create_subscription(Path, 'rover/path', self.path, 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        static = []
        for parent, child in [('husky1_map', 'husky1_odom'),
                              ('husky1_base_link', 'husky1_base_scan')]:
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.0
            static.append(transform)
        self.static_tf.sendTransform(static)
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'husky1_map'
        self.map.info.resolution = .1
        self.map.info.width = self.map.info.height = 120
        self.map.info.origin.position.x = self.map.info.origin.position.y = -6.
        self.map.info.origin.orientation.w = 1.
        self.map.data = [100 if self.occupied(-6+(x+.5)*.1, -6+(y+.5)*.1) else 0
                         for y in range(120) for x in range(120)]
        self.map_timer = self.create_timer(.5, self.publish_map)
        self.timer = self.create_timer(.05, self.tick)

    @staticmethod
    def occupied(x, y):
        # A wall lies directly between start (-2,0) and goal (2,0).
        return abs(x) < .15 and abs(y) < .9 or abs(x) > 5.8 or abs(y) > 5.8

    def publish_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)

    def command(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z
        self.last_command = time.monotonic()

    def state(self, msg):
        self.status = json.loads(msg.data)

    def path(self, msg):
        if msg.poses:
            self.paths.append([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, .1)
        self.last_tick = now
        if now-self.last_command > .5:
            self.v = self.w = 0.
        self.x += self.v*math.cos(self.yaw)*dt
        self.y += self.v*math.sin(self.yaw)*dt
        self.yaw += self.w*dt
        # Check the actual configured square footprint plus padding against the wall.
        for i in range(-5, 6):
            for j in range(-5, 6):
                dx, dy = i*.11, j*.11
                if self.occupied(self.x+dx*math.cos(self.yaw)-dy*math.sin(self.yaw),
                                 self.y+dx*math.sin(self.yaw)+dy*math.cos(self.yaw)):
                    self.collisions += 1
                    break
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id = 'husky1_odom'
        transform.child_frame_id = 'husky1_base_link'
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.z = math.sin(self.yaw/2)
        transform.transform.rotation.w = math.cos(self.yaw/2)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = transform.child_frame_id
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(odom)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'husky1_base_scan', stamp
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2*math.pi/180
        scan.range_min, scan.range_max = .05, 12.
        scan.ranges = []
        for i in range(181):
            a = self.yaw + scan.angle_min+i*scan.angle_increment
            distance = 12.
            for step in range(1, 241):
                r = step*.05
                if self.occupied(self.x+r*math.cos(a), self.y+r*math.sin(a)):
                    distance = r
                    break
            scan.ranges.append(distance)
        self.scan_pub.publish(scan)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'husky1_map'
        goal.pose.position.x = 2.
        goal.pose.orientation.z = math.sin(math.pi/4)
        goal.pose.orientation.w = math.cos(math.pi/4)
        self.goal_pub.publish(goal)


def main():
    rclpy.init()
    fixture = SyntheticRover()
    log_path = FilePath('rover_nav2_smoke.log').resolve()
    log = log_path.open('w')
    proc = subprocess.Popen([
        'ros2', 'launch', '41068_ignition_bringup', '41068_rover_navigation.launch.py',
        'start_sim:=false', 'use_sim_time:=false'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    started = time.monotonic()
    sent = False
    last_state = ''
    try:
        while time.monotonic()-started < 100:
            rclpy.spin_once(fixture, timeout_sec=.02)
            if time.monotonic()-started > 8 and not sent:
                fixture.send_goal()
                sent = True
            state = fixture.status.get('state', '')
            if state != last_state:
                print(json.dumps(fixture.status), flush=True)
                last_state = state
            if state in ('SUCCEEDED', 'FAILED'):
                break
            if proc.poll() is not None:
                raise RuntimeError('Nav2 launch exited early')
        report = {
            'state': fixture.status, 'final_pose': [fixture.x, fixture.y, fixture.yaw],
            'path_count': len(fixture.paths), 'footprint_collisions': fixture.collisions,
            'detoured': bool(fixture.paths and max(abs(y) for x, y in fixture.paths[0]) > 1.4),
            'elapsed_seconds': time.monotonic()-started,
        }
        print(json.dumps(report, indent=2), flush=True)
        if fixture.status.get('state') != 'SUCCEEDED' or fixture.collisions or not report['detoured']:
            raise AssertionError('Real Nav2 synthetic-obstacle validation failed')
        print('PASS: actual Nav2 A* and DWB reached position/heading around a wall.', flush=True)
    finally:
        os.killpg(proc.pid, signal.SIGINT) if proc.poll() is None else None
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
        log.close()
        fixture.destroy_node()
        rclpy.shutdown()
        print('Launch log: ' + str(log_path))
        if last_state != 'SUCCEEDED':
            print(log_path.read_text()[-16000:])


if __name__ == '__main__':
    main()
