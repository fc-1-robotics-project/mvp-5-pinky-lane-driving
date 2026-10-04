#!/usr/bin/env python3
"""Record one robot's lane runs from the control PC: video plus state log.

Run on the PC with ROS_DOMAIN_ID set to the robot's domain. Nothing is
published and nothing extra runs on the robot except the camera's JPEG stream.
A new run_N.mp4 / run_N.jsonl pair starts whenever a lane mission becomes
active and closes a few seconds after it ends.

    ROS_DOMAIN_ID=19 python3 lane_run_recorder.py robot2 ~/pinky_runs/20261003
"""

import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

FPS = 15.0
RAW_FPS = 5.0   # untouched camera JPEGs for labeling
TAIL_S = 3.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, robot, out_dir):
        super().__init__(f'{robot}_run_recorder')
        self.robot, self.out_dir = robot, Path(out_dir).expanduser() / robot
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.run = 0
        self.log = self.video = None
        self.started = self.last_active = self.last_frame = self.last_odom = self.last_raw = 0.
        self.state = dict(reason='-', lane='-', valid=None, mode='-', distance=0.)
        self.frames = self.rows = 0
        qos = qos_profile_sensor_data
        self.create_subscription(String, '/lane/mission_status', self.mission, 10)
        self.create_subscription(String, '/lane/diagnostics', self.diagnostic, qos)
        self.create_subscription(String, '/lane/observation', self.observation, qos)
        self.create_subscription(String, '/drive/mode_status', self.mode, qos)
        self.create_subscription(Odometry, '/odom', self.odom, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/fleet/pose', self.pose, 10)
        self.image_subscription = None
        self.create_timer(1., self.housekeeping)

    def write(self, kind, **data):
        if self.log is not None:
            self.log.write(json.dumps(dict(t=round(time.time(), 3), kind=kind, **data)) + '\n')
            self.rows += 1

    def open_run(self):
        self.run += 1
        self.started = time.time()
        self.log = (self.out_dir / f'run_{self.run}.jsonl').open('w')
        self.frames = self.rows = 0
        # Subscribe only while recording: the robot JPEG-encodes on demand.
        # NO_VIDEO=1 logs state only, so the robot does not JPEG-encode for us.
        self.image_subscription = None if os.environ.get('NO_VIDEO') else self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.image, qos_profile_sensor_data)
        print(f'[{self.robot}] run {self.run} recording started', flush=True)

    def close_run(self):
        if self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
        self.image_subscription = None
        if self.video is not None:
            self.video.release()
        self.log.close()
        print(f'[{self.robot}] run {self.run} closed: {self.frames} frames, {self.rows} rows',
              flush=True)
        self.log = self.video = None

    def mission(self, message):
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        now = time.time()
        if data.get('active'):
            if self.log is None:
                self.open_run()
            self.last_active = now
        self.state['distance'] = data.get('distance_m', 0.)
        self.write('mission', **{key: data.get(key) for key in (
            'state', 'detail', 'active', 'ready', 'readiness_reason', 'distance_m', 'mode',
            'lane_reason')})

    def housekeeping(self):
        if self.log is not None and time.time() - self.last_active > TAIL_S:
            self.close_run()

    def diagnostic(self, message):
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        self.state.update(reason=data.get('reason'), lane=data.get('lane_reason'),
                          valid=data.get('lane_valid'))
        points = data.get('path_points_m') or ()
        self.write('diag', path_points=len(points),
                   path_length=round(sum(math.dist(a, b) for a, b in zip(points, points[1:])), 3),
                   **{key: data.get(key) for key in (
                       'reason', 'lane_valid', 'lane_degraded', 'lane_reason', 'capture_age_s',
                       'observation_error', 'sensor_error', 'scan_hit', 'raw_scan_hit', 'fault')})

    def observation(self, message):
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        detections = data.get('detections', ())
        self.write('obs', classes=[d.get('class_id') for d in detections],
                   confidence=[round(d.get('confidence', 0.), 2) for d in detections],
                   traced=[len(d.get('boundary_px', ())) >= 2 for d in detections],
                   processing_time_s=data.get('processing_time_s'))

    def mode(self, message):
        try:
            self.state['mode'] = json.loads(message.data).get('mode')
        except ValueError:
            pass

    def odom(self, message):
        now = time.time()
        if now - self.last_odom < .1:
            return
        self.last_odom = now
        pose, twist = message.pose.pose, message.twist.twist
        self.write('odom', x=round(pose.position.x, 4), y=round(pose.position.y, 4),
                   yaw=round(yaw_of(pose.orientation), 4), v=round(twist.linear.x, 4),
                   w=round(twist.angular.z, 4))

    def pose(self, message):
        pose = message.pose.pose
        self.write('map_pose', x=round(pose.position.x, 3), y=round(pose.position.y, 3),
                   yaw=round(yaw_of(pose.orientation), 3))

    def image(self, message):
        now = time.time()
        if self.log is None or now - self.last_frame < 1. / FPS:
            return
        if not len(message.data):
            return
        if now - self.last_raw >= 1. / RAW_FPS:
            self.last_raw = now
            raw = self.out_dir / f'run_{self.run}_raw'
            raw.mkdir(exist_ok=True)
            (raw / f'{self.robot}_run{self.run}_{now - self.started:06.2f}s.jpg').write_bytes(
                bytes(message.data))
        frame = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.last_frame = now
        if self.video is None:
            height, width = frame.shape[:2]
            self.video = cv2.VideoWriter(str(self.out_dir / f'run_{self.run}.mp4'),
                                         cv2.VideoWriter_fourcc(*'mp4v'), FPS, (width, height))
        state = self.state
        good = state['valid'] is True and state['lane'] == 'paired'
        color = (0, 200, 0) if good else (0, 200, 255) if state['valid'] else (0, 0, 255)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 46), (0, 0, 0), -1)
        for row, text in enumerate((
                f'{self.robot} +{now - self.started:5.1f}s  {state["distance"]:.2f} m  {state["mode"]}',
                f'lane: {state["lane"]}  control: {state["reason"]}')):
            cv2.putText(frame, text, (8, 18 + 20 * row), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1,
                        cv2.LINE_AA)
        self.video.write(frame)
        self.frames += 1
        self.write('frame', index=self.frames)


def main():
    robot, out_dir = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Recorder(robot, out_dir)
    print(f'[{robot}] waiting for a lane mission; output {node.out_dir}', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.log is not None:
            node.close_run()


if __name__ == '__main__':
    main()
