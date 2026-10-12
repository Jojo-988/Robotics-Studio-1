#!/usr/bin/env python3
import json
import math
import threading
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from PIL import Image as PILImage, ImageTk
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

ROVER_CAMERA = '/husky1/camera/image'
AERIAL_CAMERA = '/parrot1/camera/image'
ROVER_ODOM = '/husky1/odom'
AERIAL_ODOM = '/parrot1/odom'
ROVER_GOAL = '/husky1/rover/goal'  # Existing custom rover goal subscriber
ROVER_STATUS = '/husky1/rover/status'
ROVER_PAUSE = '/husky1/rover/pause'
ROVER_CANCEL = '/husky1/rover/cancel'
FIRE_TOPIC = '/husky1/vision/fire_detected'
ROVER_MAP_FRAME = 'husky1_map'
PARROT_ACTION = '/parrot1/navigate_to_pose'
PARROT_MAP_FRAME = 'map'  # Check: ros2 param get /parrot1/global_costmap/global_costmap global_frame


def detect_regions(frame, mask, label, colour):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    count = 0
    for contour in contours:
        if cv2.contourArea(contour) < 800:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w < 20 or h < 30:
            continue
        count += 1
        cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 2)
        cv2.putText(frame, label, (x, max(y - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
    return count


def annotate_vegetation(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    ranges = (
        ('Healthy', (30, 70, 25), (95, 255, 210), (0, 255, 0)),
        ('Stressed', (25, 15, 60), (100, 110, 230), (0, 255, 255)),
        ('Dead', (0, 0, 100), (179, 45, 255), (0, 0, 255)),
    )
    counts = {}
    kernel = np.ones((5, 5), np.uint8)
    for label, lower, upper, colour in ranges:
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                          np.array(upper, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        counts[label] = detect_regions(frame, mask, label.upper(), colour)
    return counts


def annotate_fire(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((0, 150, 150)), np.array((10, 255, 255)))
    mask |= cv2.inRange(hsv, np.array((170, 150, 150)), np.array((179, 255, 255)))
    mask |= cv2.inRange(hsv, np.array((10, 120, 150)), np.array((35, 255, 255)))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = max(contours, key=cv2.contourArea) if contours else None
    detected = largest is not None and cv2.contourArea(largest) > 300
    if detected:
        x, y, w, h = cv2.boundingRect(largest)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 3)
        cv2.circle(frame, (x+w//2, y+h//2), 5, (255, 255, 255), -1)
        cv2.putText(frame, 'FIRE DETECTED', (x, max(y-10, 30)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return bool(detected)


class BridgeNode(Node):
    def __init__(self):
        super().__init__('forest_monitoring_gui')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {'rover': None, 'aerial': None}
        self.positions = {'rover': None, 'aerial': None}
        self.rover_status = 'Waiting for rover status'
        self.parrot_status = 'Waiting for navigation goal'
        self.fire_detected = None
        self.detection = {'rover': None, 'aerial': None}
        self.parrot_goal_handle = None
        self.parrot_goal_pending = False
        self.parrot_cancel_pending = False

        self.create_subscription(Image, ROVER_CAMERA,
                                 lambda m: self.on_image('rover', m), qos_profile_sensor_data)
        self.create_subscription(Image, AERIAL_CAMERA,
                                 lambda m: self.on_image('aerial', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, ROVER_ODOM,
                                 lambda m: self.on_odom('rover', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, AERIAL_ODOM,
                                 lambda m: self.on_odom('aerial', m), qos_profile_sensor_data)
        self.create_subscription(String, ROVER_STATUS, self.on_status, 10)
        # Fire is detected locally from the camera; do not subscribe to our own result.
        self.fire_pub = self.create_publisher(Bool, FIRE_TOPIC, 10)

        # Keep the existing custom rover interface unchanged.
        self.goal_pub = self.create_publisher(PoseStamped, ROVER_GOAL, 10)
        self.pause_client = self.create_client(SetBool, ROVER_PAUSE)
        self.cancel_client = self.create_client(Trigger, ROVER_CANCEL)

        # Parrot uses its existing Nav2 action server, not the rover custom services.
        self.parrot_nav_client = ActionClient(self, NavigateToPose, PARROT_ACTION)

    def set_parrot_status(self, message):
        with self.lock:
            self.parrot_status = message

    def on_image(self, robot, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (480, 270))
            # Process the same frame for both detectors, then display annotations.
            fire = annotate_fire(frame)
            counts = annotate_vegetation(frame)
            if robot == 'rover':
                self.fire_pub.publish(Bool(data=fire))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with self.lock:
                self.frames[robot] = frame
                self.detection[robot] = {'fire': fire, **counts,
                                         'Total': sum(counts.values())}
                if robot == 'rover':
                    self.fire_detected = fire
        except Exception as exc:
            self.get_logger().warning(f'{robot} image/detection error: {exc}')

    def on_odom(self, robot, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.positions[robot] = (p.x, p.y, p.z)

    def on_status(self, msg):
        try:
            info = json.loads(msg.data)
            status = f"{info.get('state', 'UNKNOWN')}: {info.get('reason', '')}" if isinstance(info, dict) else msg.data
        except (ValueError, TypeError):
            status = msg.data
        with self.lock:
            self.rover_status = status

    def send_goal(self, x, y, yaw_deg):
        """Existing Husky1 PoseStamped goal publisher."""
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

    def send_parrot_goal(self, x, y, z, yaw_deg):
        """Send a Parrot1 Nav2 goal. Z may be ignored by a 2D Nav2 controller."""
        if not self.parrot_nav_client.server_is_ready():
            self.set_parrot_status('Nav2 action server unavailable')
            return False
        with self.lock:
            if self.parrot_goal_pending or self.parrot_goal_handle is not None:
                self.parrot_status = 'Goal already active; cancel it before sending another'
                return False
            self.parrot_goal_pending = True
            self.parrot_status = 'Sending goal...'

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = PARROT_MAP_FRAME
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = z
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        try:
            future = self.parrot_nav_client.send_goal_async(goal)
            future.add_done_callback(self.parrot_goal_response)
            return True
        except Exception as exc:
            with self.lock:
                self.parrot_goal_pending = False
                self.parrot_status = f'Goal send failed: {exc}'
            return False

    def parrot_goal_response(self, future):
        try:
            handle = future.result()
            with self.lock:
                self.parrot_goal_pending = False
                if handle.accepted:
                    self.parrot_goal_handle = handle
                    self.parrot_status = 'Navigating (goal accepted)'
                else:
                    self.parrot_status = 'Goal rejected by Nav2'
                    self.parrot_cancel_pending = False
            if handle.accepted:
                result_future = handle.get_result_async()
                result_future.add_done_callback(self.parrot_goal_result)
                with self.lock:
                    should_cancel = self.parrot_cancel_pending
                if should_cancel:
                    self.cancel_parrot_navigation()
        except Exception as exc:
            with self.lock:
                self.parrot_goal_pending = False
                self.parrot_cancel_pending = False
                self.parrot_status = f'Goal response error: {exc}'

    def parrot_goal_result(self, future):
        try:
            status = future.result().status
            names = {
                GoalStatus.STATUS_SUCCEEDED: 'Goal reached',
                GoalStatus.STATUS_CANCELED: 'Navigation canceled',
                GoalStatus.STATUS_ABORTED: 'Navigation aborted',
            }
            message = names.get(status, f'Navigation finished (status {status})')
        except Exception as exc:
            message = f'Navigation result error: {exc}'
        with self.lock:
            self.parrot_status = message
            self.parrot_goal_handle = None
            self.parrot_goal_pending = False
            self.parrot_cancel_pending = False

    def cancel_parrot_navigation(self):
        with self.lock:
            handle = self.parrot_goal_handle
            if handle is None and self.parrot_goal_pending:
                self.parrot_cancel_pending = True
                self.parrot_status = 'Cancel queued; waiting for goal acceptance'
                return True
            if handle is None:
                return False
            self.parrot_cancel_pending = True
            self.parrot_status = 'Requesting cancellation...'
        future = handle.cancel_goal_async()
        future.add_done_callback(self.parrot_cancel_response)
        return True

    def parrot_cancel_response(self, future):
        try:
            response = future.result()
            if not response.goals_canceling:
                self.set_parrot_status('Cancel not accepted; check navigation state')
        except Exception as exc:
            self.set_parrot_status(f'Cancel request failed: {exc}')


class Dashboard:
    def __init__(self, root, node):
        self.root, self.node = root, node
        root.title('Forest Monitoring Control Station')
        root.geometry('1150x850')
        root.minsize(1000, 700)
        root.configure(bg='#15202b')

        # Scrollable layout so controls remain reachable on smaller screens.
        outer = tk.Frame(root, bg='#15202b')
        outer.pack(fill='both', expand=True)
        canvas = tk.Canvas(outer, bg='#15202b', highlightthickness=0)
        scroll = ttk.Scrollbar(outer, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        body = tk.Frame(canvas, bg='#15202b')
        window_id = canvas.create_window((0, 0), window=body, anchor='nw')
        body.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda e: canvas.itemconfigure(window_id, width=e.width))

        tk.Label(body, text='FOREST MONITORING CONTROL STATION',
                 bg='#15202b', fg='white', font=('Arial', 18, 'bold')).pack(pady=12)

        cameras = tk.Frame(body, bg='#15202b')
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
            # Each camera has its own fire and vegetation results.
            if not hasattr(self, 'detection_labels'):
                self.detection_labels = {}
            summary = tk.Label(pane, text='Detection: waiting for camera...',
                               bg='#15202b', fg='#b7c6d5', justify='left',
                               font=('Arial', 10), wraplength=480)
            summary.pack(pady=5)
            self.detection_labels[robot] = summary

        self.position_labels = {}
        positions = tk.Frame(body, bg='#15202b')
        positions.pack(pady=10)
        for robot in ('rover', 'aerial'):
            label = tk.Label(positions, text=f'{robot.title()}: waiting for odometry',
                             bg='#15202b', fg='#aee3ff', font=('Arial', 12))
            label.pack(pady=3)
            self.position_labels[robot] = label

        # Rover: retain its existing X/Y/Yaw + pause/resume + cancel controls.
        controls = tk.LabelFrame(body, text='Husky1 Navigation Goal (husky1_map)',
                                 bg='#15202b', fg='white', font=('Arial', 11))
        controls.pack(fill='x', padx=25, pady=6)
        self.entries = self.make_entries(controls, ('X (m)', 'Y (m)', 'Yaw (deg)'))
        tk.Button(controls, text='START NAVIGATION', bg='#238c55', fg='white',
                  command=self.start_navigation).grid(row=1, column=3, padx=12)
        tk.Button(controls, text='PAUSE', bg='#bd8732', fg='white',
                  command=lambda: self.pause(True)).grid(row=1, column=4, padx=5)
        tk.Button(controls, text='RESUME', bg='#3578aa', fg='white',
                  command=lambda: self.pause(False)).grid(row=1, column=5, padx=5)
        tk.Button(body, text='STOP / CANCEL ROVER NAVIGATION',
                  bg='#b72d32', fg='white', font=('Arial', 11, 'bold'),
                  command=self.cancel).pack(pady=7)

        # Parrot: same style, with altitude and Nav2 action cancellation.
        aerial_controls = tk.LabelFrame(body, text=f'Parrot1 Navigation Goal ({PARROT_MAP_FRAME})',
                                        bg='#15202b', fg='white', font=('Arial', 11))
        aerial_controls.pack(fill='x', padx=25, pady=6)
        self.aerial_entries = self.make_entries(
            aerial_controls, ('X (m)', 'Y (m)', 'Z (m)', 'Yaw (deg)'), z_default='2.0'
        )
        tk.Button(aerial_controls, text='START NAVIGATION', bg='#238c55', fg='white',
                  command=self.start_parrot_navigation).grid(row=1, column=4, padx=12)
        tk.Button(aerial_controls, text='CANCEL NAVIGATION', bg='#b72d32', fg='white',
                  command=self.cancel_parrot).grid(row=1, column=5, padx=5)
        tk.Label(aerial_controls, text='Pause / Resume requires a separate Parrot controller interface.',
                 bg='#15202b', fg='#b7c6d5').grid(row=2, column=0, columnspan=6, pady=5)

        self.status = tk.Label(body, text='Rover: waiting for status',
                               bg='#15202b', fg='white', wraplength=1000)
        self.status.pack(pady=4)
        self.parrot_status_label = tk.Label(body, text='Parrot1: waiting for goal',
                                            bg='#15202b', fg='#aee3ff', wraplength=1000)
        self.parrot_status_label.pack(pady=4)
        self.fire = tk.Label(body, text='Fire detector: waiting for data',
                             bg='#15202b', fg='white')
        self.fire.pack(pady=4)
        self.feedback = tk.Label(body, text='Ready', bg='#15202b', fg='#b7c6d5')
        self.feedback.pack(pady=5)
        self.photos = {}
        self.update()

    @staticmethod
    def make_entries(parent, names, z_default='0'):
        entries = []
        for col, name in enumerate(names):
            tk.Label(parent, text=name, bg='#15202b', fg='white').grid(
                row=0, column=col, padx=10, pady=3)
            entry = ttk.Entry(parent, width=12)
            entry.insert(0, z_default if name == 'Z (m)' else '0')
            entry.grid(row=1, column=col, padx=10, pady=5)
            entries.append(entry)
        return entries

    @staticmethod
    def read_values(entries):
        values = [float(entry.get()) for entry in entries]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Values must be finite')
        return values

    def start_navigation(self):
        try:
            self.node.send_goal(*self.read_values(self.entries))
            self.feedback.config(text='Rover goal published; check status for acceptance')
        except ValueError:
            self.feedback.config(text='Enter valid numeric rover X, Y and yaw values')

    def pause(self, paused):
        ok = self.node.request_pause(paused)
        self.feedback.config(text=('Pause' if paused else 'Resume') +
                             (' requested' if ok else ' unavailable: service not ready'))

    def cancel(self):
        ok1 = self.node.request_pause(True)
        ok2 = self.node.request_cancel()
        self.feedback.config(text='Rover stop requested; verify robot stopped' if ok1 and ok2
                             else 'WARNING: rover stop service unavailable; verify manually')

    def start_parrot_navigation(self):
        try:
            ok = self.node.send_parrot_goal(*self.read_values(self.aerial_entries))
            self.feedback.config(text='Parrot1 goal submitted; waiting for Nav2 acceptance' if ok
                                 else 'Parrot1 goal not sent; check Parrot1 status')
        except ValueError:
            self.feedback.config(text='Enter valid numeric Parrot1 X, Y, Z and yaw values')

    def cancel_parrot(self):
        ok = self.node.cancel_parrot_navigation()
        self.feedback.config(text='Parrot1 cancel requested; verify drone stopped' if ok
                             else 'No active Parrot1 goal to cancel')

    def update(self):
        with self.node.lock:
            frames = dict(self.node.frames)
            positions = dict(self.node.positions)
            status = self.node.rover_status
            parrot_status = self.node.parrot_status
            fire = self.node.fire_detected
            detection = dict(self.node.detection)
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
        for robot, result in detection.items():
            if result is not None:
                fire_text = 'FIRE DETECTED' if result['fire'] else 'No fire detected'
                self.detection_labels[robot].config(
                    text=(f"{fire_text}  |  Total regions: {result['Total']}\n"
                          f"Healthy: {result['Healthy']}   Stressed: {result['Stressed']}   "
                          f"Dead: {result['Dead']}"),
                    fg='#ff6868' if result['fire'] else '#b7c6d5')
        self.status.config(text='Rover: ' + status)
        self.parrot_status_label.config(text='Parrot1: ' + parrot_status)
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
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
