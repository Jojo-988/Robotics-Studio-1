#!/usr/bin/env python3
import json
import math
import threading
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
from cv_bridge import CvBridge

# Change these to match `ros2 topic list` in your simulation.
ROVER_CAMERA = '/husky1/camera/image'
AERIAL_CAMERA = '/parrot1/camera/image'
ROVER_ODOM = '/husky1/odom'
AERIAL_ODOM = '/parrot1/odom'
ROVER_GOAL = '/husky1/rover/goal'
ROVER_STATUS = '/husky1/rover/status'
ROVER_PAUSE = '/husky1/rover/pause'
ROVER_CANCEL = '/husky1/rover/cancel'
FIRE_TOPIC = '/husky1/vision/fire_detected'
ROVER_MAP_FRAME = 'husky1_map'


class BridgeNode(Node):
    def __init__(self):
        super().__init__('forest_monitoring_gui')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {'rover': None, 'aerial': None}
        self.positions = {'rover': None, 'aerial': None}
        self.rover_status = 'Waiting for rover status'
        self.fire_detected = None
        self.create_subscription(Image, ROVER_CAMERA,
                                 lambda m: self.on_image('rover', m),  qos_profile_sensor_data)
        self.create_subscription(Image, AERIAL_CAMERA,
                                 lambda m: self.on_image('aerial', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, ROVER_ODOM,
                                 lambda m: self.on_odom('rover', m),   qos_profile_sensor_data)
        self.create_subscription(Odometry, AERIAL_ODOM,
                                 lambda m: self.on_odom('aerial', m),  qos_profile_sensor_data)
        self.create_subscription(String, ROVER_STATUS, self.on_status, 10)
        self.create_subscription(Bool, FIRE_TOPIC, self.on_fire, 10)
        self.goal_pub = self.create_publisher(PoseStamped, ROVER_GOAL, 10)
        self.pause_client = self.create_client(SetBool, ROVER_PAUSE)
        self.cancel_client = self.create_client(Trigger, ROVER_CANCEL)

    def on_image(self, robot, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (480, 270))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with self.lock:
                self.frames[robot] = frame
        except Exception as exc:
            self.get_logger().warning(f'{robot} image error: {exc}')

    def on_odom(self, robot, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.positions[robot] = (p.x, p.y, p.z)

    def on_status(self, msg):
        try:
            info = json.loads(msg.data)
            status = f"{info.get('state', 'UNKNOWN')}: {info.get('reason', '')}"
        except (ValueError, TypeError):
            status = msg.data
        with self.lock:
            self.rover_status = status

    def on_fire(self, msg):
        with self.lock:
            self.fire_detected = msg.data

    def send_goal(self, x, y, yaw_deg):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ROVER_MAP_FRAME
        msg.pose.position.x = x
        msg.pose.position.y = y
        yaw = math.radians(yaw_deg)
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.goal_pub.publish(msg)

    def request_pause(self, paused):
        if not self.pause_client.service_is_ready():
            return False
        self.pause_client.call_async(SetBool.Request(data=paused))
        return True

    def request_cancel(self):
        if not self.cancel_client.service_is_ready():
            return False
        self.cancel_client.call_async(Trigger.Request())
        return True


class Dashboard:
    def __init__(self, root, node):
        self.root, self.node = root, node
        root.title('Forest Monitoring Control Station')
        root.geometry('1020x740')
        root.configure(bg='#15202b')
        title = tk.Label(root, text='FOREST MONITORING CONTROL STATION',
                         bg='#15202b', fg='white', font=('Arial', 18, 'bold'))
        title.pack(pady=12)

        cameras = tk.Frame(root, bg='#15202b')
        cameras.pack(fill='x', padx=14)
        self.camera_labels = {}
        for robot, label in [('rover', 'HUSKY LIVE CAMERA'), ('aerial', 'PARROT LIVE CAMERA')]:
            pane = tk.Frame(cameras, bg='#15202b')
            pane.pack(side='left', expand=True, fill='both', padx=5)
            tk.Label(pane, text=label, bg='#15202b', fg='white',
                     font=('Arial', 11, 'bold')).pack(pady=4)
            view = tk.Label(pane, text='Waiting for camera...', bg='#0a1018',
                            fg='white', width=55, height=16)
            view.pack()
            self.camera_labels[robot] = view

        self.position_labels = {}
        positions = tk.Frame(root, bg='#15202b')
        positions.pack(pady=10)
        for robot in ('rover', 'aerial'):
            label = tk.Label(positions, text=f'{robot.title()}: waiting for odometry',
                             bg='#15202b', fg='#aee3ff', font=('Arial', 12))
            label.pack(pady=3)
            self.position_labels[robot] = label

        controls = tk.LabelFrame(root, text='Rover Navigation Goal (husky1_map)',
                                 bg='#15202b', fg='white', font=('Arial', 11))
        controls.pack(fill='x', padx=25, pady=6)
        self.entries = []
        for col, name in enumerate(('X (m)', 'Y (m)', 'Yaw (deg)')):
            tk.Label(controls, text=name, bg='#15202b', fg='white').grid(
                row=0, column=col, padx=10, pady=3)
            entry = ttk.Entry(controls, width=14)
            entry.insert(0, '0')
            entry.grid(row=1, column=col, padx=10, pady=5)
            self.entries.append(entry)

        tk.Button(controls, text='START NAVIGATION', bg='#238c55', fg='white',
                  command=self.start_navigation).grid(row=1, column=3, padx=12)
        tk.Button(controls, text='PAUSE', bg='#bd8732', fg='white',
                  command=lambda: self.pause(True)).grid(row=1, column=4, padx=5)
        tk.Button(controls, text='RESUME', bg='#3578aa', fg='white',
                  command=lambda: self.pause(False)).grid(row=1, column=5, padx=5)

        tk.Button(root, text='STOP / CANCEL ROVER NAVIGATION',
                  bg='#b72d32', fg='white', font=('Arial', 12, 'bold'),
                  command=self.cancel).pack(pady=9)

        self.status = tk.Label(root, text='Rover: waiting for status',
                               bg='#15202b', fg='white', wraplength=950)
        self.status.pack(pady=4)
        self.fire = tk.Label(root, text='Fire detector: waiting for data',
                             bg='#15202b', fg='white')
        self.fire.pack(pady=4)
        self.feedback = tk.Label(root, text='Ready', bg='#15202b', fg='#b7c6d5')
        self.feedback.pack(pady=5)
        self.photos = {}
        self.update()

    def start_navigation(self):
        try:
            values = [float(entry.get()) for entry in self.entries]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Values must be finite')
            self.node.send_goal(*values)
            self.feedback.config(text='Rover goal published; check status for acceptance')
        except ValueError:
            self.feedback.config(text='Enter valid numeric X, Y and yaw values')

    def pause(self, paused):
        ok = self.node.request_pause(paused)
        self.feedback.config(text=('Pause' if paused else 'Resume') +
                             (' requested' if ok else ' unavailable: service not ready'))

    def cancel(self):
        ok1 = self.node.request_pause(True)
        ok2 = self.node.request_cancel()
        self.feedback.config(text='Rover stop requested; verify robot stopped' if ok1 and ok2
                             else 'WARNING: rover stop service unavailable; verify manually')

    def update(self):
        with self.node.lock:
            frames = dict(self.node.frames)
            positions = dict(self.node.positions)
            status = self.node.rover_status
            fire = self.node.fire_detected
        for robot, frame in frames.items():
            if frame is not None:
                photo = ImageTk.PhotoImage(PILImage.fromarray(frame))
                self.photos[robot] = photo
                self.camera_labels[robot].config(image=photo, text='', width=480, height=270)
        for robot, position in positions.items():
            if position is not None:
                self.position_labels[robot].config(
                    text=f'{robot.title()}: X={position[0]:.2f}, '
                         f'Y={position[1]:.2f}, Z={position[2]:.2f} m')
        self.status.config(text='Rover: ' + status)
        self.fire.config(text=('FIRE DETECTED' if fire else 'No fire detected')
                         if fire is not None else 'Fire detector: waiting for data',
                         fg='#ff6868' if fire else 'white')
        self.root.after(100, self.update)


def main():
    rclpy.init()
    node = BridgeNode()
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()
    root = tk.Tk()
    Dashboard(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
