#!/usr/bin/env python3
import json
import math
import os
import signal
import subprocess
import time
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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

# TOPIC LIST CONFIRGURATION, MATCH THE ROS2 TOPIC/ACTION LIST
CONFIG = {
    'rover': {
        'name': 'Husky1', 'camera': '/husky1/camera/image',
        'odom': '/husky1/odom', 'scan': '/husky1/scan',
        'goal': '/husky1/rover/goal', 'status': '/husky1/rover/status',
        'pause': '/husky1/rover/pause', 'cancel': '/husky1/rover/cancel',
        'action': '/husky1/navigate_to_pose', 'frame': 'husky1_map',
    },
    'aerial': {
        'name': 'Parrot1', 'camera': '/parrot1/camera/image',
        'odom': '/parrot1/odom', 'scan': '/parrot1/scan',
        'action': '/parrot1/navigate_to_pose', 'frame': 'parrot1_map',
    },
}
WARNING_DISTANCE = 2.0
CRITICAL_DISTANCE = 0.8


def annotate_regions(frame, mask, name, color, minimum=800):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    count = 0
    for contour in contours:
        if cv2.contourArea(contour) < minimum:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w < 20 or h < 20:
            continue
        count += 1
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, name, (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .48, color, 2)
    return count


def process_frame(frame):
    frame = cv2.resize(frame, (480, 270))
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    classes = {
        'Healthy': ((30, 70, 25), (95, 255, 210), (0, 255, 0)),
        'Stressed': ((25, 15, 60), (100, 110, 230), (0, 255, 255)),
        'Dead': ((0, 0, 100), (179, 45, 255), (0, 0, 255)),
    }
    counts = {}
    for name, (lo, hi, color) in classes.items():
        mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        counts[name] = annotate_regions(frame, mask, name.upper(), color)
    fire_mask = cv2.inRange(hsv, np.array((0, 150, 150)), np.array((10, 255, 255)))
    fire_mask |= cv2.inRange(hsv, np.array((170, 150, 150)), np.array((179, 255, 255)))
    fire_mask |= cv2.inRange(hsv, np.array((10, 120, 150)), np.array((35, 255, 255)))
    fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_OPEN, kernel)
    fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_CLOSE, kernel)
    fire = annotate_regions(frame, fire_mask, 'FIRE', (0, 0, 255), 300) > 0
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), {'fire': fire, **counts}

# ROS2 Node that bridges sensor data and navigation actions to the GUI
class BridgeNode(Node):
    def __init__(self):
        super().__init__('forest_monitoring_gui')
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.frames = {r: None for r in CONFIG}
        self.positions = {r: None for r in CONFIG}
        self.obstacles = {r: None for r in CONFIG}
        self.detections = {r: None for r in CONFIG}
        self.status = {r: 'Waiting for navigation' for r in CONFIG}
        self.last_sensor_time = {r: None for r in CONFIG}
        self.handles = {r: None for r in CONFIG}
        self.pending = {r: False for r in CONFIG}
        self.cancel_queued = {r: False for r in CONFIG}
        self.nav_clients = {}
        for robot, cfg in CONFIG.items():
            self.create_subscription(Image, cfg['camera'],
                lambda msg, r=robot: self.on_image(r, msg), qos_profile_sensor_data)
            self.create_subscription(Odometry, cfg['odom'],
                lambda msg, r=robot: self.on_odom(r, msg), qos_profile_sensor_data)
            self.create_subscription(LaserScan, cfg['scan'],
                lambda msg, r=robot: self.on_scan(r, msg), qos_profile_sensor_data)
            self.nav_clients[robot] = ActionClient(self, NavigateToPose, cfg['action'])
        rover = CONFIG['rover']
        self.create_subscription(String, rover['status'], self.on_rover_status, 10)
        self.rover_goal_pub = self.create_publisher(PoseStamped, rover['goal'], 10)
        self.rover_pause = self.create_client(SetBool, rover['pause'])
        self.rover_cancel = self.create_client(Trigger, rover['cancel'])
        self.fire_pub = self.create_publisher(Bool, '/husky1/vision/fire_detected', 10)

    # Thread-safe status update
    def set_status(self, robot, message):
        with self.lock:
            self.status[robot] = message
    
    # Callback for image messages; processes the frame and updates detection results
    def on_image(self, robot, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb, detection = process_frame(frame)
            if robot == 'rover':
                self.fire_pub.publish(Bool(data=detection['fire']))
            with self.lock:
                self.frames[robot] = rgb
                self.detections[robot] = detection
        except Exception as exc:
            self.get_logger().warning(f'{robot} image error: {exc}')

    # Callback for odometry messages; updates the robot's position
    def on_odom(self, robot, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.positions[robot] = (p.x, p.y, p.z)

    # Callback for laser scan messages; analyzes the scan data to determine obstacle proximity
    def on_scan(self, robot, msg):
        values = np.asarray(msg.ranges, dtype=float)
        if values.size == 0:
            result = ('NO VALID SCAN', None)
        else:
            angles = msg.angle_min + np.arange(values.size) * msg.angle_increment
            front = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) <= math.pi / 4
            valid = np.isfinite(values) & (values >= msg.range_min) & (values <= msg.range_max) & front
            if not np.any(valid):
                result = ('NO VALID SCAN', None)
            else:
                distance = float(np.min(values[valid]))
                level = 'CRITICAL' if distance < CRITICAL_DISTANCE else ('WARNING' if distance < WARNING_DISTANCE else 'CLEAR')
                result = (level, distance)
        with self.lock:
            self.obstacles[robot] = result
            self.last_sensor_time[robot] = self.get_clock().now().nanoseconds / 1e9
    
    # Callback for rover status messages; parses the JSON data and updates the status
    def on_rover_status(self, msg):
        try:
            data = json.loads(msg.data)
            message = f"{data.get('state', 'UNKNOWN')}: {data.get('reason', '')}" if isinstance(data, dict) else msg.data
        except (ValueError, TypeError):
            message = msg.data
        self.set_status('rover', message)
    
    # Returns a string indicating the readiness of the navigation interfaces for the specified robot
    def readiness(self, robot):
        if robot == 'rover':
            return ('custom goal subscribers: ' + str(self.rover_goal_pub.get_subscription_count()) +
                    f' | pause: {self.rover_pause.service_is_ready()}' +
                    f' | cancel: {self.rover_cancel.service_is_ready()}')
        return 'Nav2 action ready: ' + str(self.nav_clients['aerial'].server_is_ready())
    
    # Sends a navigation goal to the rover; checks for subscribers and publishes the goal if available
    def send_rover_goal(self, x, y, yaw):
        if self.rover_goal_pub.get_subscription_count() == 0:
            self.set_status('rover', 'ERROR: no /husky1/rover/goal subscriber; start rover navigation node')
            return False
        msg = self.make_pose('rover', x, y, 0.0, yaw)
        self.rover_goal_pub.publish(msg)
        self.set_status('rover', 'Goal published; awaiting rover status')
        return True
    
    # Creates a PoseStamped message for the specified robot with the given position and orientation
    def make_pose(self, robot, x, y, z, yaw):
        msg = PoseStamped()
        msg.header.frame_id = CONFIG[robot]['frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        angle = math.radians(yaw) / 2
        msg.pose.orientation.z = math.sin(angle)
        msg.pose.orientation.w = math.cos(angle)
        return msg
    
    # Sends a navigation goal to the aerial robot (Parrot); checks for action server readiness and manages goal state
    def send_aerial_goal(self, x, y, z, yaw):
        robot = 'aerial'
        if not self.nav_clients[robot].server_is_ready():
            self.set_status(robot, 'ERROR: Parrot Nav2 action server unavailable')
            return False
        with self.lock:
            if self.pending[robot] or self.handles[robot] is not None:
                self.status[robot] = 'Cancel active goal before sending another'
                return False
            self.pending[robot] = True
            self.status[robot] = 'Sending goal...'
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(robot, x, y, z, yaw)
        try:
            future = self.nav_clients[robot].send_goal_async(goal)
            future.add_done_callback(lambda f: self.on_goal_response(robot, f))
            return True
        except Exception as exc:
            with self.lock:
                self.pending[robot] = False
            self.set_status(robot, f'Goal send error: {exc}')
            return False

    # Callback for the response to a navigation goal; updates the status and manages the goal handle
    def on_goal_response(self, robot, future):
        try:
            handle = future.result()
            with self.lock:
                self.pending[robot] = False
                if handle.accepted:
                    self.handles[robot] = handle
                self.status[robot] = 'Goal accepted; navigating' if handle.accepted else 'Goal rejected'
                queued = self.cancel_queued[robot]
            if handle.accepted:
                handle.get_result_async().add_done_callback(lambda f: self.on_goal_result(robot, f))
                if queued:
                    self.cancel_aerial()
        except Exception as exc:
            with self.lock:
                self.pending[robot] = False
                self.cancel_queued[robot] = False
            self.set_status(robot, f'Goal response error: {exc}')
    
    # Callback for the result of a navigation goal; updates the status and clears the goal handle
    def on_goal_result(self, robot, future):
        try:
            code = future.result().status
            message = {GoalStatus.STATUS_SUCCEEDED: 'Goal reached',
                       GoalStatus.STATUS_CANCELED: 'Goal canceled',
                       GoalStatus.STATUS_ABORTED: 'Goal aborted (check Nav2 logs)'}.get(code, f'Finished: {code}')
        except Exception as exc:
            message = f'Goal result error: {exc}'
        with self.lock:
            self.handles[robot] = None
            self.pending[robot] = False
            self.cancel_queued[robot] = False
            self.status[robot] = message
    
    # Cancels the current navigation goal for the aerial robot
    def cancel_aerial(self):
        with self.lock:
            handle = self.handles['aerial']
            if handle is None and self.pending['aerial']:
                self.cancel_queued['aerial'] = True
                self.status['aerial'] = 'Cancel queued until goal accepted'
                return True
            if handle is None:
                return False
            self.cancel_queued['aerial'] = True
            self.status['aerial'] = 'Requesting cancel...'
        future = handle.cancel_goal_async()
        future.add_done_callback(self.on_cancel_response)
        return True
    
    # updates the status based on whether the cancel was accepted
    def on_cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.set_status('aerial', 'Cancel rejected by action server')
        except Exception as exc:
            self.set_status('aerial', f'Cancel error: {exc}')
    
    # Pauses or resumes the rover's navigation based on the 'paused' parameter
    def pause_rover(self, paused):
        if not self.rover_pause.service_is_ready():
            self.set_status('rover', 'Pause service unavailable')
            return False
        self.rover_pause.call_async(SetBool.Request(data=paused)).add_done_callback(
            lambda f: self.service_response('rover', f, 'Pause' if paused else 'Resume'))
        return True

    def cancel_rover(self):
        if not self.rover_cancel.service_is_ready():
            self.set_status('rover', 'Cancel service unavailable')
            return False
        self.rover_cancel.call_async(Trigger.Request()).add_done_callback(
            lambda f: self.service_response('rover', f, 'Cancel'))
        return True

    # Handles the response from a service call; updates the status based on the result
    def service_response(self, robot, future, name):
        try:
            response = future.result()
            self.set_status(robot, f'{name}: {"accepted" if response.success else "failed"} {response.message}')
        except Exception as exc:
            self.set_status(robot, f'{name} service error: {exc}')

# GUI Dashboard class that creates the main window and manages the interface
class Dashboard:
    BG = '#15202b'
    FG = '#e8f1f8'
    MUTED = '#b7c6d5'

    def __init__(self, root, node):
        self.root, self.node = root, node
        root.title('Forest Monitoring Control Station')
        root.geometry('1200x920')
        root.minsize(980, 650)
        root.configure(bg=self.BG)
        self.photos = {}
        self.camera_labels = {}
        self.detect_labels = {}
        self.position_labels = {}
        self.obstacle_labels = {}
        self.status_labels = {}
        self.ready_labels = {}
        self.inputs = {}
        self.patrol_process = None
        self.patrol_lock = threading.RLock()
        self.patrol_status = 'IDLE'
        self.patrol_stopping = False
        outer = tk.Frame(root, bg=self.BG)
        outer.pack(fill='both', expand=True)
        canvas = tk.Canvas(outer, bg=self.BG, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        body = tk.Frame(canvas, bg=self.BG)
        wid = canvas.create_window((0, 0), window=body, anchor='nw')
        body.bind('<Configure>', lambda _: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda e: canvas.itemconfigure(wid, width=e.width))
        canvas.bind_all('<MouseWheel>', lambda e: canvas.yview_scroll(-int(e.delta / 120), 'units'))
        tk.Label(body, text='FOREST MONITORING CONTROL STATION', bg=self.BG,
                 fg=self.FG, font=('Arial', 18, 'bold')).pack(pady=12)
        cameras = tk.Frame(body, bg=self.BG)
        cameras.pack(fill='x', padx=12)
        for robot, cfg in CONFIG.items():
            pane = tk.Frame(cameras, bg=self.BG)
            pane.pack(side='left', fill='both', expand=True, padx=6)
            self.label(pane, cfg['name'].upper() + ' CAMERA', bold=True).pack(pady=3)
            view = tk.Label(pane, text='Waiting for camera...', bg='#0a1018', fg=self.FG,
                            width=60, height=15)
            view.pack()
            self.camera_labels[robot] = view
            self.detect_labels[robot] = self.label(pane, 'Detection: waiting...', wrap=490)
            self.detect_labels[robot].pack(pady=4)
            self.position_labels[robot] = self.label(pane, 'Position: waiting...')
            self.position_labels[robot].pack(pady=3)
            self.obstacle_labels[robot] = self.label(pane, 'Obstacle: waiting for LiDAR...', bold=True)
            self.obstacle_labels[robot].pack(pady=4)
        for robot, cfg in CONFIG.items():
            frame = tk.LabelFrame(body, text=cfg['name'] + ' Navigation (' + cfg['frame'] + ')',
                                  bg=self.BG, fg=self.FG, font=('Arial', 11, 'bold'))
            frame.pack(fill='x', padx=20, pady=8)
            names = ('X (m)', 'Y (m)', 'Yaw (deg)') if robot == 'rover' else ('X (m)', 'Y (m)', 'Z (m)', 'Yaw (deg)')
            entries = []
            for i, name in enumerate(names):
                self.label(frame, name).grid(row=0, column=i, padx=5)
                entry = ttk.Entry(frame, width=12)
                entry.insert(0, '2.0' if name == 'Z (m)' else '0')
                entry.grid(row=1, column=i, padx=5, pady=5)
                entries.append(entry)
            self.inputs[robot] = entries
            tk.Button(frame, text='SEND GOAL', bg='#238c55', fg='white',
                      command=lambda r=robot: self.send_goal(r)).grid(row=1, column=len(names), padx=7)
            tk.Button(frame, text='CANCEL', bg='#b72d32', fg='white',
                      command=lambda r=robot: self.cancel(r)).grid(row=1, column=len(names)+1, padx=7)
            if robot == 'rover':
                tk.Button(frame, text='PAUSE', command=lambda: self.node.pause_rover(True)).grid(row=2, column=0, pady=5)
                tk.Button(frame, text='RESUME', command=lambda: self.node.pause_rover(False)).grid(row=2, column=1, pady=5)
            self.ready_labels[robot] = self.label(frame, 'Checking interface...', wrap=1050)
            self.ready_labels[robot].grid(row=3, column=0, columnspan=len(names)+2, sticky='w', padx=6, pady=4)
            self.status_labels[robot] = self.label(frame, 'Waiting for navigation...', wrap=1050)
            self.status_labels[robot].grid(row=4, column=0, columnspan=len(names)+2, sticky='w', padx=6, pady=4)
        patrol = tk.LabelFrame(body, text='PARROT AUTONOMOUS COVERAGE PATROL',
                               bg=self.BG, fg=self.FG, font=('Arial', 11, 'bold'))
        patrol.pack(fill='x', padx=20, pady=8)
        self.label(patrol, 'Altitude (m)').grid(row=0, column=0, padx=8, pady=8)
        self.patrol_altitude = ttk.Entry(patrol, width=10)
        self.patrol_altitude.insert(0, '15.0')
        self.patrol_altitude.grid(row=0, column=1, padx=5)
        self.patrol_repeat = tk.BooleanVar(value=False)
        tk.Checkbutton(patrol, text='Repeat patrol', variable=self.patrol_repeat,
                       bg=self.BG, fg=self.FG, selectcolor=self.BG,
                       activebackground=self.BG, activeforeground=self.FG).grid(row=0, column=2, padx=8)
        self.patrol_start_button = tk.Button(patrol, text='START PATROL', bg='#238c55',
                                             fg='white', command=self.start_patrol)
        self.patrol_start_button.grid(row=0, column=3, padx=8)
        self.patrol_stop_button = tk.Button(patrol, text='STOP PATROL', bg='#b72d32',
                                            fg='white', command=self.stop_patrol)
        self.patrol_stop_button.grid(row=0, column=4, padx=8)
        self.patrol_label = self.label(patrol, 'Patrol: IDLE', wrap=1050)
        self.patrol_label.grid(row=1, column=0, columnspan=5, sticky='w', padx=8, pady=6)
        self.feedback = self.label(body, 'Ready. LiDAR warnings do not stop robots. Parrot Nav2 may ignore Z altitude.', wrap=1080)
        self.feedback.pack(pady=9)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.update()

    # Setup the label widget with specified text, boldness, and wrap length
    def label(self, parent, text, bold=False, wrap=0):
        return tk.Label(parent, text=text, bg=self.BG, fg=self.MUTED,
                        font=('Arial', 10, 'bold' if bold else 'normal'), wraplength=wrap)
    
    # Checks if the patrol process is currently active (running and not stopped)
    def patrol_active(self):
        with self.patrol_lock:
            return self.patrol_process is not None and self.patrol_process.poll() is None
    
    # Starts the coverage patrol by launching a ROS2 launch file with specified parameters; checks for existing patrol or manual goals
    def start_patrol(self):
        if self.patrol_active():
            self.feedback.config(text='Patrol already running. Stop it before restarting.')
            return
        with self.node.lock:
            manual_busy = (self.node.pending['aerial'] or
                           self.node.handles['aerial'] is not None)
        if manual_busy:
            self.feedback.config(text='Cancel the active Parrot manual goal before starting patrol.')
            return
        try:
            altitude = float(self.patrol_altitude.get())
            if not math.isfinite(altitude) or altitude <= 0:
                raise ValueError('altitude must be positive and finite')
        except ValueError:
            self.feedback.config(text='Enter a positive finite patrol altitude.')
            return
        command = [
            'ros2', 'launch', '41068_ignition_bringup',
            '41068_coverage_patrol.launch.py', 'mode:=attach',
            f'altitude:={altitude}',
            f'repeat:={str(self.patrol_repeat.get()).lower()}',
        ]
        try:
            with self.patrol_lock:
                self.patrol_process = subprocess.Popen(command, start_new_session=True)
                self.patrol_stopping = False
                self.patrol_status = f'STARTING (PID {self.patrol_process.pid})'
            self.feedback.config(text='Coverage patrol launch started. Check terminal logs and mission readiness.')
        except (OSError, ValueError) as exc:
            with self.patrol_lock:
                self.patrol_status = f'LAUNCH ERROR: {exc}'
            self.feedback.config(text=f'Unable to launch patrol: {exc}')

    # Stops the coverage patrol by sending a termination signal to the process
    def stop_patrol(self):
        with self.patrol_lock:
            proc = self.patrol_process
            if proc is None or proc.poll() is not None or self.patrol_stopping:
                return
            self.patrol_stopping = True
            self.patrol_status = 'STOPPING (not an emergency stop)'
        def shutdown():
            try:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=2)
            except (ProcessLookupError, OSError, subprocess.TimeoutExpired):
                pass
            with self.patrol_lock:
                if self.patrol_process is proc:
                    self.patrol_status = 'STOPPED — verify drone motion has ceased'
                    self.patrol_stopping = False
        threading.Thread(target=shutdown, daemon=True).start()

    # Sends a navigation goal to the specified robot; checks for active patrol before sending a manual goal
    def send_goal(self, robot):
        if robot == 'aerial' and self.patrol_active():
            self.feedback.config(text='Stop Coverage Patrol before sending a manual Parrot goal.')
            return
        try:
            values = [float(e.get()) for e in self.inputs[robot]]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('non-finite input')
        except ValueError:
            self.feedback.config(text='Enter valid finite numeric coordinates.')
            return
        if robot == 'rover':
            ok = self.node.send_rover_goal(*values)
        else:
            ok = self.node.send_aerial_goal(*values)
        self.feedback.config(text=(('Goal submitted to Nav2. Parrot Z movement requires altitude-capable controller.' if robot == 'aerial' else 'Goal submitted. Check status.') if ok else 'Goal NOT sent. Check status.'))

    def cancel(self, robot):
        if robot == 'rover':
            ok = self.node.cancel_rover()
        else:
            ok = self.node.cancel_aerial()
        self.feedback.config(text='Cancel requested; verify robot motion stops.' if ok else 'Cancel unavailable/no active goal.')

    def update(self):
        with self.node.lock:
            frames = dict(self.node.frames)
            positions = dict(self.node.positions)
            detections = dict(self.node.detections)
            obstacles = dict(self.node.obstacles)
            statuses = dict(self.node.status)
            last_times = dict(self.node.last_sensor_time)
        now = self.node.get_clock().now().nanoseconds / 1e9
        for robot in CONFIG:
            if frames[robot] is not None:
                photo = ImageTk.PhotoImage(PILImage.fromarray(frames[robot]))
                self.photos[robot] = photo
                self.camera_labels[robot].config(image=photo, text='', width=480, height=270)
            if positions[robot] is not None:
                x, y, z = positions[robot]
                self.position_labels[robot].config(text=f'Position: X={x:.2f}, Y={y:.2f}, Z={z:.2f} m')
            if detections[robot] is not None:
                d = detections[robot]
                self.detect_labels[robot].config(text=f"Fire: {'DETECTED' if d['fire'] else 'not detected'} | "
                    f"Healthy: {d['Healthy']} | Stressed: {d['Stressed']} | Dead: {d['Dead']}")
            if obstacles[robot] is not None:
                level, dist = obstacles[robot]
                if last_times[robot] is not None and now - last_times[robot] > 2.0:
                    level = 'STALE SCAN'
                color = {'CLEAR': '#00df88', 'WARNING': '#ffb347', 'CRITICAL': '#ff5252'}.get(level, '#aaaaaa')
                info = f'{dist:.2f} m' if dist is not None else 'distance unknown'
                self.obstacle_labels[robot].config(text=f'Obstacle: {level} | {info}', fg=color)
            self.status_labels[robot].config(text='Navigation: ' + statuses[robot])
            self.ready_labels[robot].config(text='Interface: ' + self.node.readiness(robot))
        with self.patrol_lock:
            proc = self.patrol_process
            if proc is not None and proc.poll() is not None and not self.patrol_stopping:
                self.patrol_status = f'EXITED (code {proc.returncode})'
            elif proc is not None and proc.poll() is None and not self.patrol_stopping:
                self.patrol_status = f'LAUNCH RUNNING (PID {proc.pid}); check patrol logs for flight status'
            patrol_status = self.patrol_status
        self.patrol_label.config(text='Patrol: ' + patrol_status)
        self.patrol_start_button.config(state='disabled' if self.patrol_active() or self.patrol_stopping else 'normal')
        self.patrol_stop_button.config(state='normal' if self.patrol_active() and not self.patrol_stopping else 'disabled')
        self.root.after(150, self.update)

    def close(self):
        self.stop_patrol()
        self.root.destroy()


def main():
    rclpy.init()
    node = BridgeNode()
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()
    try:
        root = tk.Tk()
        Dashboard(root, node)
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
