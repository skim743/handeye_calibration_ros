#!/usr/bin/env python3
# -*-coding:utf8-*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
import cv2
import numpy as np
import json
import datetime
import sys
import termios
import tty
import time
import os

def clear_input_buffer():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def create_dir(dir=""):
    if dir == "" or dir == "/":
        exit(-1)
    if not os.path.exists(dir) or not os.path.isdir(dir):
        try:
            if os.path.exists(dir):
                os.remove(dir)
            os.makedirs(dir)
        except IOError:
            print("cannot create directory: " + dir)
            exit(-2)

def create_pose_from_martix(translation_matrix, rotation_matrix) -> Pose:
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat().tolist()  # [x, y, z, w]

    pose = Pose()
    pose.position.x = translation_matrix[0][0]
    pose.position.y = translation_matrix[1][0]
    pose.position.z = translation_matrix[2][0]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

class PoseMartix():
    def __init__(self, pose_input: Pose, check_mode=False, mode='eye_in_hand'):
        position = pose_input.position
        orientation = pose_input.orientation
        self.position_list = [position.x, position.y, position.z]
        self.orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation = Rotation.from_quat(self.orientation_list)
        self.rpy_list = [rotation.as_euler('xyz').tolist()]
        self.translation_matrix = np.array(self.position_list)
        self.rotation_matrix = rotation.as_matrix()
        if check_mode and mode == 'eye_to_hand':
            T = np.eye(4)
            T[:3, :3] = self.rotation_matrix
            T[:3, 3] = self.translation_matrix
            T_inv = np.linalg.inv(T)
            self.rotation_matrix = T_inv[:3, :3]
            self.translation_matrix = T_inv[:3, 3]

    def as_T(self):
        T = np.eye(4)
        T[:3, :3] = self.rotation_matrix
        T[:3, 3] = np.ravel(self.translation_matrix)
        return T

HANDEYE_METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}

def motion_information(piper_poses):
    # Accumulated information of the relative motions A_i = T_0^-1 T_i.
    # Rotation: sum of a a^T (a = rotation vector). Translation: sum of (R_A - I)^T (R_A - I).
    # Their minimum eigenvalues never decrease as samples are added.
    I_R = np.zeros((3, 3))
    I_t = np.zeros((3, 3))
    T0_inv = np.linalg.inv(piper_poses[0].as_T())
    for pose in piper_poses[1:]:
        R_A = (T0_inv @ pose.as_T())[:3, :3]
        a = Rotation.from_matrix(R_A).as_rotvec()
        I_R += np.outer(a, a)
        D = R_A - np.eye(3)
        I_t += D.T @ D
    return I_R, I_t

def consistency(piper_poses, marker_poses, R_x, t_x):
    # Eye in hand: base->target; eye to hand: gripper->target. Must be constant across samples.
    X = np.eye(4)
    X[:3, :3] = R_x
    X[:3, 3] = np.ravel(t_x)
    Ts = [p.as_T() @ X @ m.as_T() for p, m in zip(piper_poses, marker_poses)]
    t = np.array([T[:3, 3] for T in Ts])
    t_err = np.linalg.norm(t - t.mean(axis=0), axis=1)
    rots = Rotation.from_matrix(np.array([T[:3, :3] for T in Ts]))
    r_err = np.degrees((rots.mean().inv() * rots).magnitude())
    return t_err, r_err

class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration")

        # Declare parameters
        self.declare_parameter('mode', 'eye_in_hand')
        self.declare_parameter('min_num', 10)
        self.declare_parameter('piper_topic', '/piper_ctrl_node/end_pose')
        self.declare_parameter('marker_topic', '/aruco_single/pose')
        self.declare_parameter('result_save_path', './result')
        self.declare_parameter('joint_topic', '/joint_states_single')
        # "/aruco_single/pose", "/piper_ctrl_node/end_pose"

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.min_num = self.get_parameter('min_num').get_parameter_value().integer_value
        self.piper_topic = self.get_parameter('piper_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.result_save_path = self.get_parameter('result_save_path').get_parameter_value().string_value
        self.joint_topic = self.get_parameter('joint_topic').get_parameter_value().string_value

        print(f"mode: {self.mode}")
        print(f"min_num: {self.min_num}")
        print(f"piper_topic: {self.piper_topic}")
        print(f"marker_topic: {self.marker_topic}")
        print(f"joint_topic: {self.joint_topic}")
        print(f"result_save_path: {self.result_save_path}")

        self.piper_poses = []
        self.marker_poses = []
        self.joint_states = []
        self.filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    def save_samples(self):
        # Rewritten after every sample/undo so a crash or 'c' exit keeps the data
        samples = [dict(
            joint_names = list(joint.name),
            joint_positions = list(joint.position),
            piper_pose = dict(position = piper.position_list, orientation = piper.orientation_list),
            marker_pose = dict(position = marker.position_list, orientation = marker.orientation_list),
        ) for joint, piper, marker in zip(self.joint_states, self.piper_poses, self.marker_poses)]
        create_dir(self.result_save_path)
        with open(f"{self.result_save_path}/{self.filename}_samples.json", 'w+') as json_file:
            json.dump(dict(mode = self.mode, joint_topic = self.joint_topic, samples = samples), json_file, indent=4)

    def solve_handeye(self, method):
        R_gripper2base = [data.rotation_matrix for data in self.piper_poses]
        t_gripper2base = [data.translation_matrix for data in self.piper_poses]
        R_target2cam = [data.rotation_matrix for data in self.marker_poses]
        t_target2cam = [data.translation_matrix for data in self.marker_poses]
        return cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=method)

    def report_quality(self):
        n = len(self.piper_poses)
        if n < 2:
            return
        I_R, I_t = motion_information(self.piper_poses)
        w_R, v_R = np.linalg.eigh(I_R)
        w_t, _ = np.linalg.eigh(I_t)
        # Relative motions are taken w.r.t. the first sample; eye_to_hand passes inverted piper poses, so the axis lands in the base frame
        axis_frame = "first-sample gripper frame" if self.mode == 'eye_in_hand' else "base frame"
        print(f"rot info min eig: {w_R[0]:.4f} rad^2, weakest axis ({axis_frame}): {np.round(v_R[:, 0], 2).tolist()}")
        print(f"trans info min eig: {w_t[0]:.4f}")
        if n < 3:
            return
        R_x, t_x = self.solve_handeye(cv2.CALIB_HAND_EYE_TSAI)
        t_err, r_err = consistency(self.piper_poses, self.marker_poses, R_x, t_x)
        t_rms = np.sqrt(np.mean(t_err ** 2))
        r_rms = np.sqrt(np.mean(r_err ** 2))
        print(f"consistency RMS: {t_rms * 1000:.1f} mm, {r_rms:.2f} deg | last sample: {t_err[-1] * 1000:.1f} mm, {r_err[-1]:.2f} deg")
        if n >= 5 and (t_err[-1] > 2 * t_rms or r_err[-1] > 2 * r_rms):
            print("WARNING: last sample is an outlier, consider 'd' to undo")

    def report_quality_safe(self):
        # get_poses() exits on any exception, so a failed quality check must not propagate
        try:
            self.report_quality()
        except Exception as e:
            print(f"quality check failed: {e}")

    def final_quality(self):
        quality = {}
        for name, method in HANDEYE_METHODS.items():
            try:
                R_x, t_x = self.solve_handeye(method)
                t_err, r_err = consistency(self.piper_poses, self.marker_poses, R_x, t_x)
                quality[name] = dict(
                    position = np.ravel(t_x).tolist(),
                    orientation = Rotation.from_matrix(R_x).as_quat().tolist(),
                    trans_rms_mm = float(np.sqrt(np.mean(t_err ** 2)) * 1000),
                    rot_rms_deg = float(np.sqrt(np.mean(r_err ** 2))),
                )
                print(f"{name:>10}: t = {np.round(np.ravel(t_x) * 1000, 1).tolist()} mm, "
                      f"RMS {quality[name]['trans_rms_mm']:.1f} mm / {quality[name]['rot_rms_deg']:.2f} deg")
            except cv2.error as e:
                print(f"{name:>10}: failed ({e})")
        return quality

    def process_handeye(self):
        R_gripper2base = [data.rotation_matrix for data in self.piper_poses]
        t_gripper2base = [data.translation_matrix for data in self.piper_poses]
        R_target2cam = [data.rotation_matrix for data in self.marker_poses]
        t_target2cam = [data.translation_matrix for data in self.marker_poses]
        ret_R, ret_t = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)
        return PoseMartix(create_pose_from_martix(ret_t, ret_R))

    def run(self):
        if self.mode != "eye_in_hand" and self.mode != "eye_to_hand":
            raise ValueError("'mode' must be either eye_in_hand or eye_to_hand")
        if self.min_num < 5:
            raise ValueError("the minimum number of samples must be at least 5")
        self.get_poses()
        result_pose = self.process_handeye()
        result = dict(
            position = result_pose.position_list,
            orientation = result_pose.orientation_list,
            rpy = result_pose.rpy_list
        )

        print("calibration result:")
        print("")
        print(json.dumps(result, indent=4))
        print("")
        print("method comparison (consistency RMS):")
        result['quality'] = self.final_quality()
        print("")
        create_dir(self.result_save_path)
        filename = self.filename
        with open(f"{self.result_save_path}/{filename}_calibration.json", 'w+') as json_file:
            json.dump(result, json_file, indent=4)
            print(f"writing to {self.result_save_path}/{filename}_calibration.json")

    def get_poses(self):
        count = 1
        while True:
            print(f"\n------- sample {count} ------")
            if count == 1:
                menu_str = "input(Enter-sample):"
            elif count <= self.min_num:
                menu_str = "input(Enter-sample, d-undo):"
            else:
                menu_str = "input(Enter-sample, d-undo, q-calculate and exit):"
            try:
                clear_input_buffer()
                user_input = input(menu_str+" ")
                if user_input == '':
                    count += 1
                    marker_pose_raw, piper_pose_raw, joint_state = self.subscribe_message()
                    piper_pose = PoseMartix(piper_pose_raw, True, self.mode)
                    marker_pose = PoseMartix(marker_pose_raw.pose, False, self.mode)
                    print("---")
                    print(f"piper: {piper_pose.position_list}")
                    print(f"marker: {marker_pose.position_list}")
                    self.piper_poses.append(piper_pose)
                    self.marker_poses.append(marker_pose)
                    self.joint_states.append(joint_state)
                    print(f"joints: {np.round(joint_state.position, 4).tolist()}")
                    self.save_samples()
                    self.report_quality_safe()
                elif count > 1 and user_input == 'd':
                    count -= 1
                    self.piper_poses.pop()
                    self.marker_poses.pop()
                    self.joint_states.pop()
                    self.save_samples()
                elif user_input == 'c':
                    print("exit")
                    exit(0)
                elif count > self.min_num and user_input == 'q':
                    print("calculating calibration result")
                    break
                else:
                    print("invalid input")
            except Exception as e:  # Exception as e
                #exit(0)
                #pass
                #print(e)
                print("\nprogram error or interrupted, exiting")
                exit(-1)	# -3

    def callback(self, msg):
        self.get_msg=True
        self.call_msg = msg

    def subscribe_message(self):
        sys.stdout.write("wait marker data... ")
        sys.stdout.flush()
        marker_pose_raw = self.subcribe_one_message(self.marker_topic, PoseStamped)
        sys.stdout.write("[ok]\n")

        sys.stdout.write("wait piper data... ")
        sys.stdout.flush()
        piper_pose_raw = self.subcribe_one_message(self.piper_topic, Pose)
        sys.stdout.write("[ok]\n")

        sys.stdout.write("wait joint data... ")
        sys.stdout.flush()
        joint_state = self.subcribe_one_message(self.joint_topic, JointState)
        sys.stdout.write("[ok]\n")

        return marker_pose_raw, piper_pose_raw, joint_state

    def subcribe_one_message(self, topic, msg_type):
        self.get_msg=False
        sub = self.create_subscription(msg_type, topic, self.callback, 1)
        while not self.get_msg:
            rclpy.spin_once(self)
            time.sleep(0.05) 
        msg = self.call_msg
        self.destroy_subscription(sub)
        return msg

def main(args=None):
    rclpy.init(args=args)
    calibration_node = HandEyeCalibrationNode()
    calibration_node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
