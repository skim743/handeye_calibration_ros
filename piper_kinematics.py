"""
PiperKinematics: forward/inverse kinematics and the arm's physical constants.

Owns the numbers the rest of the stack treats as ground truth -- per-joint
position and speed limits, link masses and COM offsets, and the Denavit-
Hartenberg table (with the optional 2-degree j2/j3 offset variant selected by
dh_is_offset). Link 6's d is extended by 120 mm so the DH chain ends at the
gripper tip rather than the flange.

computeFK() walks the DH chain and returns the pose of every link, computeIK()
solves for joint angles with damped least squares over computeJacobian(),
clipping to the joint limits each iteration and raising if it fails to converge
within _ik_max_iterations. Note the unit convention at the boundary: FK returns
mm and degrees, IK takes and returns m and radians -- callers in ARMLabABC do
the conversion.

Author: Soobum Kim <skim743@gatech.edu>
Created: 2026-03-05
Updated: 2026-08-24
"""

import math
import numpy as np
from typing import (
    Literal,
)


class PiperKinematics():
    # Joint Position Limits
    JOINT1_POSITION_LIMITS = (-2.6179, 2.6179)
    JOINT2_POSITION_LIMITS = (0.0, 3.14)
    JOINT3_POSITION_LIMITS = (-2.967, 0)
    JOINT4_POSITION_LIMITS = (-1.745, 1.745)
    JOINT5_POSITION_LIMITS = (-1.22, 1.22)
    JOINT6_POSITION_LIMITS = (-2.09439, 2.09439)
    GRIPPER_POSITION_LIMITS = (0.0, 0.07)
    JOINT_POSITION_LIMITS = [JOINT1_POSITION_LIMITS, 
                            JOINT2_POSITION_LIMITS, 
                            JOINT3_POSITION_LIMITS, 
                            JOINT4_POSITION_LIMITS, 
                            JOINT5_POSITION_LIMITS, 
                            JOINT6_POSITION_LIMITS, 
                            GRIPPER_POSITION_LIMITS]

    # Joint Speed Limits
    JOINT1_SPEED_LIMITS = 3.1416
    JOINT2_SPEED_LIMITS = 3.1416 #3.4034
    JOINT3_SPEED_LIMITS = 3.1416
    JOINT4_SPEED_LIMITS = 3.9270
    JOINT5_SPEED_LIMITS = 3.9270
    JOINT6_SPEED_LIMITS = 3.9270
    GRIPPER_SPEED_LIMITS = 0.7 # A higher number will cause the gripper to quickly open to a certain extent depending on the gripper speed limit.
    JOINT_SPEED_LIMITS = [JOINT1_SPEED_LIMITS,
                        JOINT2_SPEED_LIMITS,
                        JOINT3_SPEED_LIMITS,
                        JOINT4_SPEED_LIMITS,
                        JOINT5_SPEED_LIMITS,
                        JOINT6_SPEED_LIMITS,
                        GRIPPER_SPEED_LIMITS]

    # Joint Torque Limits in Nm. Based on piper_sdk documentation
    JOINT1_TORQUE_LIMITS = 32.0 # 8 / 0.25
    JOINT2_TORQUE_LIMITS = 32.0 # 8 / 0.25
    JOINT3_TORQUE_LIMITS = 32.0 # 8 / 0.25
    JOINT4_TORQUE_LIMITS = 6.4 # 8 / 1.25
    JOINT5_TORQUE_LIMITS = 6.4 # 8 / 1.25
    JOINT6_TORQUE_LIMITS = 6.4 # 8 / 1.25
    GRIPPER_TORQUE_LIMITS = 5.0
    JOINT_TORQUE_LIMITS = [JOINT1_TORQUE_LIMITS,
                        JOINT2_TORQUE_LIMITS,
                        JOINT3_TORQUE_LIMITS,
                        JOINT4_TORQUE_LIMITS,
                        JOINT5_TORQUE_LIMITS,
                        JOINT6_TORQUE_LIMITS,
                        GRIPPER_TORQUE_LIMITS]

    # Link Masses in kg
    LINK_MASS = [0.71, 1.16, 0.5, 0.38, 0.383, 0.507] # in kg

    # Link Center of Mass Offsets in the local link frame (in meters)
    LINK_COM_OFFSET = [
        [ 0.00032,             -0.00041,             -0.00348],              # link1
        [ 0.218520972713,      -0.0085852173386,      0.00126040729334],     # link2
        [-0.0199774942153,     -0.148102572711,      -0.000529929793331],    # link3
        [ 0.000138396039085,   -0.000760039077106,   -0.00391730848923],     # link4
        [ 0.000188967153084,   -0.0610570887264,     -0.00227005089449],     # link5
        [-0.000121189328298,    7.37563066241e-05,   -0.0822743064555],      # link6+gripper_base+link7+link8
    ]

    def __init__(self, dh_is_offset: Literal[0x00, 0x01] = 0x01):
        self.RADIAN = 180 / math.pi
        self.PI = math.pi
        # Denavit-Hartenberg parameters for each link
        # _a: link lengths
        # _alpha: link twists
        # _theta: joint angles
        # _d: link offsets
        self._a     = [0     , 0                      , 285.03                   , -21.98        , 0             , 0          ]
        self._alpha = [0     , -self.PI / 2           , 0                        , self.PI / 2   , -self.PI / 2  , self.PI / 2]
        self._theta = [0     , -self.PI * 174.22 / 180, -100.78 / 180 * self.PI  , 0             , 0             , 0          ]
        # self._d     = [123   , 0                      , 0                        , 250.75        , 0             , 91         ]
        self._d     = [123   , 0                      , 0                        , 250.75        , 0             , 211         ] # Link 6 offset increased by 120mm to account for gripper length
        self.init_pos   = [55.0  , 0.0                    , 205.0                    , 0.0           , 85.0          , 0.0] # unit xyz-mm, rpy-degree
        # if j2, j3 offset 2°
        if(dh_is_offset == 0x01):
            self._a     = [0     , 0                      , 285.03                   , -21.98        , 0             , 0          ]
            self._alpha = [0     , -self.PI / 2           , 0                        , self.PI / 2   , -self.PI / 2  , self.PI / 2]
            self._theta = [0     , -self.PI * 172.22 / 180, -102.78 / 180 * self.PI  , 0             , 0             , 0          ]
            self._d     = [123   , 0                      , 0                        , 250.75        , 0             , 211         ] # Link 6 offset increased by 120mm to account for gripper length
            self.init_pos   = [56.128, 0.0                    , 213.266                  , 0.0           , 85.0          , 0.0] # unit xyz-mm, rpy-degree

        self._ik_max_iterations = 50
        self._ik_position_tolerance = 1e-3 # in meters
        self._ik_orientation_tolerance = 1e-2 # in radians
        self._ik_damping_factor = 0.1 # Damping factor for the damped least squares method in IK

        self.joint_position_limits = list(self.JOINT_POSITION_LIMITS)

        self._link_mass = np.array(self.LINK_MASS)  # Convert to numpy array for easier calculations
        self._com_h = [np.array(offset + [1.0]) for offset in self.LINK_COM_OFFSET]  # Convert to homogeneous coordinates
        self._tril = np.tril(np.ones((6, 6)))  # Lower triangular matrix to zero out upper triangular part of Jacobian for each link

    def __MatrixToeula(self, T):
        '''
        Convert a transformation matrix to Euler angles (roll, pitch, yaw).
        T: 4x4 transformation matrix
        '''
        Pos = [0.0] * 6
        # Extract position (x, y, z)
        Pos[0] = T[3]  # x position
        Pos[1] = T[7]  # y position
        Pos[2] = T[11] # z position
        # Calculate Euler angles (roll, pitch, yaw) based on rotation matrix
        if T[8] < -1 + 0.0001:
            Pos[4] = self.PI / 2 * self.RADIAN  # pitch (beta)
            Pos[5] = 0
            Pos[3] = math.atan2(T[1], T[5]) * self.RADIAN # roll (alpha)
        elif T[8] > 1 - 0.0001:
            Pos[4] = -self.PI / 2 * self.RADIAN # pitch (beta)
            Pos[5] = 0
            Pos[3] = -math.atan2(T[1], T[5]) * self.RADIAN # roll (alpha)
        else:
            # General case for Euler angles computation
            _bt = math.atan2(-T[8], math.sqrt(T[0] * T[0] + T[4] * T[4])) # pitch (beta)
            Pos[4] = _bt * self.RADIAN
            Pos[5] = math.atan2(T[4] / math.cos(_bt), T[0] / math.cos(_bt)) * self.RADIAN # yaw (gamma)
            Pos[3] = math.atan2(T[9] / math.cos(_bt), T[10] / math.cos(_bt)) * self.RADIAN # roll (alpha)

        return Pos

    def __MatMultiply(self, matrix1, matrix2, m, l, n):
        '''
        Multiply two matrices
        matrix1: first matrix
        matrix2: second matrix
        m: number of rows in matrix1
        l: number of columns in matrix1 (rows in matrix2)
        n: number of columns in matrix2
        '''
        matrixOut = [0.0] * (m * n)
        for i in range(m):
            for j in range(n):
                tmp = 0.0
                for k in range(l):
                    tmp += matrix1[l * i + k] * matrix2[n * k + j]
                matrixOut[n * i + j] = tmp
        return matrixOut
    
    def __LinkTransformtion(self, alpha, a, theta, d):
        '''
        Compute the transformation matrix for a single link using the Denavit-Hartenberg parameters
        alpha: link twist
        a: link length
        theta: joint angle
        d: link offset
        '''
        # Precompute trigonometric functions for efficiency
        calpha = math.cos(alpha)
        salpha = math.sin(alpha)
        ctheta = math.cos(theta)
        stheta = math.sin(theta)

        T = [0.0] * 16 # 4x4 transformation matrix
        T[0] = ctheta
        T[1] = -stheta
        T[2] = 0
        T[3] = a

        T[4] = stheta * calpha
        T[5] = ctheta * calpha
        T[6] = -salpha
        T[7] = -salpha * d

        T[8] = stheta * salpha
        T[9] = ctheta * salpha
        T[10] = calpha
        T[11] = calpha * d

        T[12] = 0
        T[13] = 0
        T[14] = 0
        T[15] = 1

        return T
    
    def __vec_to_mat(self, vec):
        """
        Convert a 16-element list to a 4x4 numpy matrix.
        """
        mat = np.array(vec, dtype=float).reshape((4, 4))
        return mat
    
    def __normalize_joint_angles(self, joint_angles):
        '''
        Normalize joint angles to be within the range of [-pi, pi]
        joint_angles: list of joint angles in radians
        '''
        normalized_angles = []
        for angle in joint_angles:
            normalized_angle = (angle + np.pi) % (2 * np.pi) - np.pi
            normalized_angles.append(normalized_angle)
        return normalized_angles
    
    def euler_to_rotation_matrix(self, euler_angles):
        '''
        Convert Euler angles to a rotation matrix
        euler_angles: list of Euler angles [roll, pitch, yaw] in radians
        '''
        roll, pitch, yaw = euler_angles
        sr = math.sin(roll)
        cr = math.cos(roll)
        sp = math.sin(pitch)
        cp = math.cos(pitch)
        sy = math.sin(yaw)
        cy = math.cos(yaw)

        rotation_matrix = np.array([
            [cy*cp,   cy*sp*sr - sy*cr,   cy*sp*cr + sy*sr],
            [sy*cp,   sy*sp*sr + cy*cr,   sy*sp*cr - cy*sr],
            [-sp,     cp*sr,              cp*cr            ]
        ])
        return rotation_matrix

    def computeFK(self, cur_j):
        '''
        Calculate Forward Kinematics for a given joint configuration
        cur_j: list of joint angles
        Returns the positions and Euler angles for each link
        '''
        # Initialize transformation matrices
        _Rt = [[0.0] * 16 for _ in range(6)]

        # Compute the individual transformation matrices
        for i in range(6):
            c_theta = cur_j[i] + self._theta[i]
            _Rt[i] = self.__LinkTransformtion(self._alpha[i], self._a[i], c_theta, self._d[i])

        # Multiply transformation matrices
        T = [_Rt[0]]
        for i in range(5):
            T.append(self.__MatMultiply(T[i], _Rt[i+1], 4, 4, 4))

        # Extract Euler angles for each transformation
        j_pos = []
        for i in range(6):
            j_pos.append(self.__MatrixToeula(T[i]))   # Euler angles for link i+1

        return j_pos
    
    def computeJacobian_ee(self, cur_j):
        '''
        Calculate the geometric Jacobian matrix for a given joint configuration
        cur_j: list of joint angles
        Returns the Jacobian matrix
        '''
        # Initialize transformation matrices
        _Rt = [[0.0] * 16 for _ in range(6)]

        # Compute the individual transformation matrices
        for i in range(6):
            c_theta = cur_j[i] + self._theta[i]
            _Rt[i] = self.__LinkTransformtion(self._alpha[i], self._a[i], c_theta, self._d[i])

        # Multiply transformation matrices
        T = [_Rt[0]] # Start with identity matrix for base frame
        for i in range(5):
            T.append(self.__MatMultiply(T[i], _Rt[i+1], 4, 4, 4))

        # Initialize Jacobian matrix
        J = np.zeros((6, 6)) # 6x6 Jacobian for 6-DOF manipulator

        # Compute the joint origin and z-axis in the base frame
        joint_origins = []
        z_axes = []
        for i in range(6):
            T_i = self.__vec_to_mat(T[i])
            T_i[:3, 3] *= 1e-3  # Convert mm to m for position
            joint_origins.append(T_i[:3,3])
            z_axes.append(T_i[:3,2])

        # Compute the Jacobian matrix
        for i in range(6):
            J[0:3, i] = np.cross(z_axes[i], (joint_origins[5] - joint_origins[i])) # Linear component
            J[3:6, i] = z_axes[i] # Angular component

        return J

    def computeJacobian_com(self, cur_j):
            '''
            Calculate the Jacobian matrix for the center of mass for all 6 links, stacked
            cur_j: list of joint angles
            g: gravitational acceleration
            Returns J with shape (6, 6, 6):
            J[i]         -> link i's 6x6 COM Jacobian
            J[i][:3, j]  -> linear  block: z_j x (p_ci - o_j)
            J[i][3:, j]  -> angular block: z_j
            J[i][:, j]   == 0 for j > i   (joint j+1 is distal to link i)
            '''

            # Initialize transformation matrices
            _Rt = [[0.0] * 16 for _ in range(6)]
    
            # Compute the individual transformation matrices
            for i in range(6):
                c_theta = cur_j[i] + self._theta[i]
                _Rt[i] = self.__LinkTransformtion(self._alpha[i], self._a[i], c_theta, self._d[i])
            
            # Multiply transformation matrices
            T = [_Rt[0]]
            for i in range(5):
                T.append(self.__MatMultiply(T[i], _Rt[i+1], 4, 4, 4))
    
            # Compute the joint origin, z-axis, and position of each link's center of mass in the base frame
            O = np.zeros((6, 3)) # Joint origins (o_j)
            Z = np.zeros((6, 3)) # Z axes (z_j)
            P = np.zeros((6, 3)) # COM positions (p_ci)

            for i in range(6):
                # com_local = np.array(self.LINK_COM_OFFSET[i] + [1.0])
                T_i = self.__vec_to_mat(T[i])
                T_i[:3, 3] *= 1e-3  # Convert mm to m for position
                O[i] = T_i[:3, 3]  # joint origin in base frame
                Z[i] = T_i[:3, 2]  # z-axis in base frame
                P[i] = (T_i @ self._com_h[i])[:3]  # COM position in base frame

            # Compute the Jacobian matrix for the center of mass
            D = P[:, None, :] - O[None, :, :]  # Vector from joint origins to COM positions. D[i, j] = p_ci - o_j
            J = np.zeros((6, 6, 6)) # Initialize Jacobian for all links
            J[:, :3, :] = np.cross(Z[None, :, :], D).transpose(0, 2, 1)
            J[:, 3:, :] = Z.T # Angular velocity Jacobian for center of mass
            J *= self._tril[:, None, :] # Zeros out the upper triangular part of the Jacobian for each link, since joint j+1 is distal to link i

            return J

    def computeGravityTorque(self, cur_j, g=9.81):
        '''
        Compute the gravity torques for the given joint angles and gravitational acceleration.
        '''
        # Compute the Jacobian matrix for the center of mass
        J = self.computeJacobian_com(cur_j)

        # Compute gravity torques
        tau = np.zeros(7) # 6-DOF + gripper
        tau[:6] = -g*(self._link_mass @ J[:, 2, :]) # Compute gravity torques for all joints using the Jacobian and link masses

        return tau
    
    def computeIK(self, target_pose, initial_guess):
        '''
        Calculate Inverse Kinematics
        '''
        flag_success = False
        joint_positions = initial_guess.copy()
        for iteration in range(self._ik_max_iterations):
            current_pose_raw = self.computeFK(joint_positions[0:6])[-1] # Get end-effector pose
            current_pose = np.empty(6, dtype=np.float64)
            current_pose[0:3] = np.array(current_pose_raw[0:3])*0.001 # Convert mm to m for position
            current_pose[3:6] = np.array(current_pose_raw[3:6])*np.pi/180 # Convert degrees to radians for orientation
            ee_pose_error = [0.0]*6
            ee_pose_error[0:3] = np.array(target_pose[0:3]) - current_pose[0:3] # Position error
            # Orientation error (axis-angle representation)
            R_error = self.euler_to_rotation_matrix(target_pose[3:6]) @ self.euler_to_rotation_matrix(current_pose[3:6]).T
            # Convert to axis-angle vector (axis * angle)
            angle = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
            if abs(angle) < 1e-10:
                orientation_error = np.zeros(3)
            else:
                orientation_error = angle / (2 * np.sin(angle)) * np.array([
                    R_error[2,1] - R_error[1,2],
                    R_error[0,2] - R_error[2,0],
                    R_error[1,0] - R_error[0,1]
                ])
            ee_pose_error[3:6] = orientation_error

            # Check convergence
            if np.linalg.norm(ee_pose_error[0:3]) < self._ik_position_tolerance and np.linalg.norm(ee_pose_error[3:6]) < self._ik_orientation_tolerance:
                flag_success = True
                break

            J = self.computeJacobian_ee(joint_positions[0:6])
            delta_pose = J.T @ np.linalg.solve((J@J.T + self._ik_damping_factor**2*np.eye(6)), ee_pose_error) # Damped Least Squares

            # Update joint positions while clipping them to be within valid joint limits
            for i in range(6):
                new_joint_position = joint_positions[i] + delta_pose[i] # Update joint positions
                # new_joint_position += delta_pose[i] # Update joint positions
                joint_positions[i] = np.clip(new_joint_position, self.joint_position_limits[i][0], self.joint_position_limits[i][1])

            # Normalize joint angles to be within [-pi, pi]
            joint_positions[0:6] = self.__normalize_joint_angles(joint_positions[0:6])

        # Ensure angle continuity
        for i in range(6):
            if abs(joint_positions[i] - initial_guess[i]) > math.pi:
                if joint_positions[i] > initial_guess[i]:
                    joint_positions[i] -= 2 * math.pi
                else:
                    joint_positions[i] += 2 * math.pi
                
                joint_positions[i] = np.clip(joint_positions[i], self.joint_position_limits[i][0], self.joint_position_limits[i][1])

        if not flag_success:
            # ee_pose_error = np.array(target_pose) - np.array(self.computeFK(joint_positions[0:6])[-1])
            # print("IK did not converge within the maximum number of iterations. Final pose error:", ee_pose_error)
            raise ValueError("IK did not converge within the maximum number of iterations. Final pose error: {}".format(ee_pose_error))

        return joint_positions
