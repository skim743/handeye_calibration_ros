#!/usr/bin/env python3
# -*-coding:utf8-*-
# Replays the joint configurations of a *_samples.json (from handeye_calibration_record)
# in position-control mode and runs the same collection + solve pipeline.

import rclpy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
import numpy as np
import json
import time

from handeye_calibration_ros.handeye_calibration_record import HandEyeCalibrationNode, PoseMartix, create_dir

def pose_from_dict(d) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = d['position']
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = d['orientation']
    return pose

def pose_difference(a: PoseMartix, b: PoseMartix):
    # Uses the raw (never inverted) pose lists, so it is valid in both modes
    t_diff = np.linalg.norm(np.array(a.position_list) - np.array(b.position_list))
    r_diff = (Rotation.from_quat(a.orientation_list).inv() * Rotation.from_quat(b.orientation_list)).magnitude()
    return float(t_diff * 1000), float(np.degrees(r_diff))

class HandEyeCalibrationAutoNode(HandEyeCalibrationNode):
    def __init__(self):
        super().__init__()

        self.declare_parameter('samples_file', '')
        self.declare_parameter('command_topic', '/joint_states')
        self.declare_parameter('joint_tolerance', 0.01)    # rad, per joint
        self.declare_parameter('settle_time', 1.5)         # s within tolerance before capturing
        self.declare_parameter('move_timeout', 30.0)       # s per pose
        self.declare_parameter('capture_timeout', 5.0)     # s per topic message
        self.declare_parameter('ignore_joints', ['gripper', 'joint7'])  # gripper is not checked for arrival
        self.declare_parameter('approach_offset', 0.0)     # rad; nonzero = pass through target + offset first
        self.declare_parameter('confirm_start', True)      # false = start moving without the Enter prompt (ros2 launch has no stdin)

        self.samples_file = self.get_parameter('samples_file').get_parameter_value().string_value
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.joint_tolerance = self.get_parameter('joint_tolerance').get_parameter_value().double_value
        self.settle_time = self.get_parameter('settle_time').get_parameter_value().double_value
        self.move_timeout = self.get_parameter('move_timeout').get_parameter_value().double_value
        self.capture_timeout = self.get_parameter('capture_timeout').get_parameter_value().double_value
        self.ignore_joints = list(self.get_parameter('ignore_joints').get_parameter_value().string_array_value)
        self.approach_offset = self.get_parameter('approach_offset').get_parameter_value().double_value
        self.confirm_start = self.get_parameter('confirm_start').get_parameter_value().bool_value

        print(f"samples_file: {self.samples_file}")
        print(f"command_topic: {self.command_topic}")
        print(f"joint_tolerance: {self.joint_tolerance} rad, settle_time: {self.settle_time} s")
        print(f"approach_offset: {self.approach_offset} rad")

        self.cmd_pub = self.create_publisher(JointState, self.command_topic, 10)
        self.feedback = None
        self.feedback_sub = self.create_subscription(JointState, self.joint_topic, self.feedback_callback, 10)
        self.replay = []

    def feedback_callback(self, msg):
        self.feedback = msg

    def subcribe_one_message(self, topic, msg_type):
        # Same as the parent, but gives up after capture_timeout instead of waiting forever
        self.get_msg = False
        sub = self.create_subscription(msg_type, topic, self.callback, 1)
        start = time.time()
        while not self.get_msg:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start > self.capture_timeout:
                self.destroy_subscription(sub)
                raise TimeoutError(f"no message on {topic} within {self.capture_timeout} s")
        msg = self.call_msg
        self.destroy_subscription(sub)
        return msg

    def move_to(self, names, positions, settle_time=None):
        settle_time = self.settle_time if settle_time is None else settle_time
        target = dict(zip(names, positions))
        checked = [n for n in names if n not in self.ignore_joints]
        cmd = JointState()
        cmd.name = list(names)
        cmd.position = [float(p) for p in positions]

        start = time.time()
        last_pub = 0.0
        reached_at = None
        err = float('inf')
        while rclpy.ok() and time.time() - start < self.move_timeout:
            now = time.time()
            if now - last_pub >= 0.1:
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.cmd_pub.publish(cmd)
                last_pub = now
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.feedback is None:
                continue
            current = dict(zip(self.feedback.name, self.feedback.position))
            missing = [n for n in checked if n not in current]
            if missing:
                raise KeyError(f"joints {missing} not in {self.joint_topic} feedback")
            err = max(abs(current[n] - target[n]) for n in checked)
            if err < self.joint_tolerance:
                if reached_at is None:
                    reached_at = now
                elif now - reached_at >= settle_time:
                    return True, err
            else:
                reached_at = None
        return False, err

    def get_poses(self):
        with open(self.samples_file) as json_file:
            recorded = json.load(json_file)
        if 'samples' not in recorded:
            print(f"{self.samples_file} has no 'samples' key; pass a *_samples.json, not a *_calibration.json")
            exit(-1)
        samples = recorded['samples']
        self.num_recorded = len(samples)
        if recorded.get('mode') != self.mode:
            print(f"WARNING: samples were recorded in mode '{recorded.get('mode')}', running in '{self.mode}'")

        print(f"\n{len(samples)} poses loaded. The arm must be enabled and NOT in teaching mode.")
        print("Clear the workspace; the arm moves in joint space to each pose.")
        if self.confirm_start and input("input(Enter-start, anything else-abort): ") != '':
            print("aborted")
            exit(0)

        for i, sample in enumerate(samples, start=1):
            print(f"\n------- pose {i}/{len(samples)} ------")
            if self.approach_offset != 0.0:
                # Every arm joint arrives from the same side, so gear backlash is taken up consistently
                approach = [p if n in self.ignore_joints else p + self.approach_offset
                            for n, p in zip(sample['joint_names'], sample['joint_positions'])]
                reached, err = self.move_to(sample['joint_names'], approach, settle_time=0.0)
                if not reached:
                    print(f"WARNING: approach pose not reached (max joint error {err:.4f} rad), skipping")
                    continue
            reached, err = self.move_to(sample['joint_names'], sample['joint_positions'])
            if not reached:
                print(f"WARNING: pose not reached (max joint error {err:.4f} rad), skipping")
                continue
            try:
                marker_pose_raw, piper_pose_raw, joint_state = self.subscribe_message()
            except TimeoutError as e:
                print(f"\nWARNING: {e}, skipping")
                continue
            piper_pose = PoseMartix(piper_pose_raw, True, self.mode)
            marker_pose = PoseMartix(marker_pose_raw.pose, False, self.mode)
            self.piper_poses.append(piper_pose)
            self.marker_poses.append(marker_pose)
            self.joint_states.append(joint_state)
            self.save_samples()

            # Same joints as the recording: differences are control/feedback repeatability + detection noise
            piper_diff = pose_difference(PoseMartix(pose_from_dict(sample['piper_pose'])), piper_pose)
            marker_diff = pose_difference(PoseMartix(pose_from_dict(sample['marker_pose'])), marker_pose)
            self.replay.append(dict(pose = i, max_joint_error = err, piper_diff = piper_diff, marker_diff = marker_diff))
            print(f"vs recorded | piper: {piper_diff[0]:.1f} mm, {piper_diff[1]:.2f} deg | "
                  f"marker: {marker_diff[0]:.1f} mm, {marker_diff[1]:.2f} deg")
            self.report_quality_safe()

        if len(self.piper_poses) < 3:
            print("fewer than 3 poses collected, cannot calibrate")
            exit(-1)
        self.report_replay()

    def report_replay(self):
        piper = np.array([r['piper_diff'] for r in self.replay])
        marker = np.array([r['marker_diff'] for r in self.replay])
        print(f"\n{len(self.replay)}/{self.num_recorded} poses replayed and compared with the recording")
        print(f"piper  vs recorded: mean {piper[:, 0].mean():.1f} mm / {piper[:, 1].mean():.2f} deg, "
              f"max {piper[:, 0].max():.1f} mm / {piper[:, 1].max():.2f} deg")
        print(f"marker vs recorded: mean {marker[:, 0].mean():.1f} mm / {marker[:, 1].mean():.2f} deg, "
              f"max {marker[:, 0].max():.1f} mm / {marker[:, 1].max():.2f} deg")
        create_dir(self.result_save_path)
        with open(f"{self.result_save_path}/{self.filename}_replay.json", 'w+') as json_file:
            json.dump(dict(samples_file = self.samples_file, approach_offset = self.approach_offset, poses = self.replay), json_file, indent=4)

def main(args=None):
    rclpy.init(args=args)
    calibration_node = HandEyeCalibrationAutoNode()
    calibration_node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
