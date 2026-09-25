# Hand-Eye Calibration Package Based on ROS2

|UBUNTU|ROS|PYTHON|OPENCV|STATE|
|---|---|---|---|---|
|![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![python](https://img.shields.io/badge/python-3.10-blue.svg)|![opencv](https://img.shields.io/badge/opencv-4.5.0-blue.svg)|![Pass](https://img.shields.io/badge/Pass-green.svg)|

## Hand-Eye Calibration

By collecting multiple sets of end-effector poses from the robotic arm and camera-recognized calibration board poses as input, two types of calibration results can be obtained:
- Eye in hand: The transformation matrix between the robotic arm's end-effector and the camera.
- Eye to hand: The transformation matrix between the robotic arm's base and the camera.

## 1. Installation
### 1.1 Dependencies
```
$ sudo apt install libopencv-dev python3-opencv
$ sudo apt-get install ros-$ROS_DISTRO-tf-transformations
```

### 1.2 Drivers
For testing, we used the `Original ArUco` dictionary calibration board and the `piper` robotic arm.

- [Online Calibration Board Generator]( https://chev.me/arucogen/)
- [Camera Recognition](https://github.com/pal-robotics/aruco_ros/tree/humble-devel)
- [Piper Robotic Arm Program](https://github.com/agilexrobotics/piper_ros/tree/humble)


### 1.3 Build from source
```
$ mkdir -p ros2_ws/src
$ cd ros2_ws/src
$ git clone 
$ cd ..
$ colcon build --symlink-install
```

## 2. Directly run
### 2.1 Start Camera
- Start the camera node according to the actual setup.

### 2.2 Start Robot Arm
```
$ ros2 launch piper start_single_piper.launch.py can_port:=can0
```
- The robotic arm must be in **teaching mode**.

### 2.3 Start Camera Recognition
```
$ ros2 launch aruco_ros single.launch
```
- Need to correct the **marker's size** and **id**, as well as the **image topic** and **frame_id**.

### 2.4 Start Camera Calibration 
When collecting data, it is recommended to move the robot slowly and collect more angle information.

Usage: `enter` collects a set of data, `d` deletes a set of data, `q` calculates the calibration result and prints it out,  `c` exit.

#### 2.4.1 Eye in Hand
```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_in_hand
```
- Collection Instructions: The camera is fixed at the end of the robotic arm, and the calibration board is placed flat on the table. Operate the robotic arm to allow the camera to recognize the calibration board on the table.

#### 2.4.2 Eye to Hand

```
$ ros2 run handeye_calibration_ros handeye_calibration --ros-args -p piper_topic:=/piper_ctrl_node/end_pose -p marker_topic:=/aruco_single/pose  -p mode:=eye_to_hand
```
- Collection Instructions: The camera is fixed at a specific position, and the calibration board is fixed at the end of the robotic arm. Operate the robotic arm to allow the camera to recognize the calibration board at the arm's end.


#### 2.4.3 Parameters

|param|type|default|Description|
|---|---|---|---|
|mode|string|eye_in_hand|hand-eye calibration mode|
|min_num|int|10|minimum number of data sets|
|piper_topic|string|piper_ctrl_node/end_pose|robotic arm's end-effector(geometry_msgs/Pose)|
|marker_topic|string|aruco_single/pose|camera-recognized calibration board pose topic（geometry_msgs/PoseStamped）|

### 2.5 Launch Files
Launch files wrap the record and auto-replay nodes with the eye-to-hand defaults used on the Piper setup. Rebuild after adding or editing them (`colcon build`), then override any argument with `name:=value`.

Use two terminals: the bringup launch file starts the sensors and arm, and the record/auto launch file runs the interactive calibration node (keyboard input only works when the calibration node is launched on its own).

#### 2.5.0 Bringup (`handeye_bringup.launch.py`)
Starts the RealSense camera (color remapped to `/stereo/left/*`), the arm driver (`piper start_single_piper.launch.py`) and ArUco detection (`aruco_ros single.launch.py`), replacing sections 2.1–2.3. Start this first, then the record or auto launch file in a second terminal.
```
$ ros2 launch handeye_calibration_ros handeye_bringup.launch.py
```

|argument|default|Description|
|---|---|---|
|mode|eye_to_hand|selects the camera `_usb_port_id`: 2-9 for eye_to_hand, 2-10 for eye_in_hand|
|eye|left|passed to aruco_ros|
|marker_id|100|passed to aruco_ros|
|marker_size|0.1|passed to aruco_ros (m)|

`result_save_path` defaults to `./result`, so output files land relative to the directory you launch from.

#### 2.5.1 Record (`handeye_calibration_record.launch.py`)
Manual collection in teaching mode, same keys as 2.4. Each sample's joint state is also saved to `<timestamp>_samples.json` for later replay.
```
$ ros2 launch handeye_calibration_ros handeye_calibration_record.launch.py
```
Equivalent to:
```
$ ros2 run handeye_calibration_ros handeye_calibration_record --ros-args -p mode:=eye_to_hand -p piper_topic:=/end_pose
```

|argument|default|
|---|---|
|mode|eye_to_hand|
|piper_topic|/end_pose|

#### 2.5.2 Auto replay (`handeye_calibration_auto.launch.py`)
Drives the arm through the joint configurations in a `*_samples.json` (position control, arm enabled and **not** in teaching mode), recollects every sample, and solves. Press `Enter` to start once the workspace is clear.
```
$ ros2 launch handeye_calibration_ros handeye_calibration_auto.launch.py samples_file:=/handeye_ws/result/<timestamp>_samples.json
```
Equivalent to:
```
$ ros2 run handeye_calibration_ros handeye_calibration_auto --ros-args -p samples_file:=<share>/calibration_files/2026-09-23_19-04-23_samples.json -p mode:=eye_to_hand -p piper_topic:=/end_pose -p settle_time:=5.0 -p approach_offset:=0.0
```

|argument|default|Description|
|---|---|---|
|samples_file|`<share>/calibration_files/2026-09-23_19-04-23_samples.json`|recorded samples to replay (must be `*_samples.json`, not `*_calibration.json`)|
|mode|eye_to_hand|hand-eye calibration mode|
|piper_topic|/end_pose|robotic arm's end-effector pose topic|
|settle_time|5.0|s the joints must stay within tolerance before capturing|
|approach_offset|0.0|rad; nonzero makes each joint pass through target + offset first so backlash is taken up from one side|

`<share>` is `$(ros2 pkg prefix handeye_calibration_ros)/share/handeye_calibration_ros`. `calibration_files/*.json` is installed there by `setup.py`, so rebuild after adding a new samples file.

Other node parameters (`command_topic`, `joint_tolerance`, `move_timeout`, `capture_timeout`, `marker_topic`, `joint_topic`) keep their node defaults; pass them with `ros2 run ... --ros-args -p` if needed. Output: `<timestamp>_samples.json`, `<timestamp>_calibration.json`, and `<timestamp>_replay.json` (replayed vs recorded pose differences).
