#!/usr/bin/env python3
# -*-coding:utf8-*-
# Touch test for an eye_to_hand calibration: put the gripper tip on the center of a
# marker lying in the workspace (teach mode), press Enter, and compare where the camera
# says the marker is (through the calibration) with where the arm says the tip is.
#
# python3 handeye_touch_test.py --ros-args -p calibration_file:=<..._calibration.json>

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
import numpy as np
import datetime
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from piper_kinematics import PiperKinematics

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

class HandEyeTouchTestNode(Node):
    def __init__(self):
        super().__init__("handeye_touch_test")

        self.declare_parameter('calibration_file', '')
        self.declare_parameter('method', 'park')
        self.declare_parameter('marker_topic', '/aruco_single/pose')
        self.declare_parameter('joint_topic', '/joint_states_single')
        self.declare_parameter('marker_frames', 30)       # detections averaged per point
        self.declare_parameter('capture_timeout', 5.0)    # s
        self.declare_parameter('result_save_path', './result')

        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.method = self.get_parameter('method').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.joint_topic = self.get_parameter('joint_topic').get_parameter_value().string_value
        self.marker_frames = self.get_parameter('marker_frames').get_parameter_value().integer_value
        self.capture_timeout = self.get_parameter('capture_timeout').get_parameter_value().double_value
        self.result_save_path = self.get_parameter('result_save_path').get_parameter_value().string_value

        with open(self.calibration_file) as json_file:
            calibration = json.load(json_file)
        # 'quality' holds every method (handeye_calibration_record); fall back to the top-level result
        cam2base = calibration.get('quality', {}).get(self.method, calibration)
        # eye_to_hand calibration is camera -> base: p_base = R p_cam + t
        self.R_cam2base = Rotation.from_quat(cam2base['orientation']).as_matrix()
        self.t_cam2base = np.array(cam2base['position'])

        print(f"calibration_file: {self.calibration_file} ({self.method if cam2base is not calibration else 'top-level result'})")
        print(f"cam2base t: {np.round(self.t_cam2base * 1000, 1).tolist()} mm")
        print(f"marker_topic: {self.marker_topic}, joint_topic: {self.joint_topic}, marker_frames: {self.marker_frames}")

        self.kinematics = PiperKinematics()  # default DH set matches the firmware's end_pose
        self.points = []
        self.filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    def collect(self, topic, msg_type, count):
        msgs = []
        sub = self.create_subscription(msg_type, topic, msgs.append, 10)
        start = time.time()
        while len(msgs) < count:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start > self.capture_timeout:
                self.destroy_subscription(sub)
                raise TimeoutError(f"got {len(msgs)}/{count} messages on {topic} within {self.capture_timeout} s")
        self.destroy_subscription(sub)
        return msgs[:count]

    def measure(self):
        markers = self.collect(self.marker_topic, PoseStamped, self.marker_frames)
        p_cam = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in markers])
        joint = self.collect(self.joint_topic, JointState, 1)[0]
        positions = dict(zip(joint.name, joint.position))
        q = [positions[n] for n in JOINT_NAMES]

        p_marker = self.R_cam2base @ p_cam.mean(axis=0) + self.t_cam2base
        p_tip = np.array(self.kinematics.computeFK(q)[-1][:3]) / 1000  # mm -> m
        return dict(
            joints = q,
            marker_base = p_marker.tolist(),
            tip_base = p_tip.tolist(),
            miss = (p_marker - p_tip).tolist(),
            marker_jitter_mm = float(np.linalg.norm(p_cam.std(axis=0)) * 1000),
        )

    def summary(self):
        miss = np.array([p['miss'] for p in self.points]) * 1000
        norm = np.linalg.norm(miss, axis=1)
        print(f"\n{len(self.points)} points")
        print(f"miss: mean {norm.mean():.1f} mm, RMS {np.sqrt(np.mean(norm ** 2)):.1f} mm, max {norm.max():.1f} mm")
        print(f"mean miss vector (systematic part): {np.round(miss.mean(axis=0), 1).tolist()} mm")
        print(f"horizontal (xy) mean {np.linalg.norm(miss[:, :2], axis=1).mean():.1f} mm, vertical (z) mean {np.abs(miss[:, 2]).mean():.1f} mm")

    def save(self):
        os.makedirs(self.result_save_path, exist_ok=True)
        path = f"{self.result_save_path}/{self.filename}_touch_test.json"
        with open(path, 'w+') as json_file:
            json.dump(dict(calibration_file = self.calibration_file, method = self.method, points = self.points), json_file, indent=4)
        return path

    def run(self):
        print("\nTeach mode: put the gripper tip on the test marker's center and hold still.")
        while True:
            user_input = input(f"\n[{len(self.points)} points] input(Enter-measure, d-undo, q-summary and exit): ")
            if user_input == '':
                try:
                    point = self.measure()
                except (TimeoutError, KeyError) as e:
                    print(f"measurement failed: {e}")
                    continue
                self.points.append(point)
                miss = np.array(point['miss']) * 1000
                print(f"marker (camera): {np.round(np.array(point['marker_base']) * 1000, 1).tolist()} mm")
                print(f"tip (arm FK):    {np.round(np.array(point['tip_base']) * 1000, 1).tolist()} mm")
                print(f"miss: {np.round(miss, 1).tolist()} mm, |miss| = {np.linalg.norm(miss):.1f} mm "
                      f"(marker jitter {point['marker_jitter_mm']:.1f} mm)")
                self.save()
            elif user_input == 'd' and self.points:
                self.points.pop()
                self.save()
            elif user_input == 'q':
                if self.points:
                    self.summary()
                    print(f"writing to {self.save()}")
                break
            else:
                print("invalid input")

def main(args=None):
    rclpy.init(args=args)
    node = HandEyeTouchTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
