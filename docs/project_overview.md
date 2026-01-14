# Mevius Project Overview (ROS 2 Jazzy)

This document provides an overview of the MEVIUS quadruped robot project, including its hardware, software components, dependencies, and operational instructions. This project aims to create a quadruped robot that can be easily constructed using e-commerce components, sheet metal welding, and machining.

This document is specifically tailored for the **ROS 2 Jazzy** environment.

## 1. Project Links

*   [Project Page](https://haraduka.github.io/mevius-hardware)
*   [arXiv](https://arxiv.org/abs/2409.14721)
*   [YouTube](https://www.youtube.com/watch?v=XXJ4EK3Y4zQ)

## 2. Hardware Information

All hardware components are detailed in the following Google Drive link:
*   [Google Drive - Hardware Components](https://drive.google.com/drive/folders/18i6CWtUG8fkY0rCcbjN5WiUYItjIX0WM?usp=sharing)

**Motor Configuration (T-Motor AK70-10):**
*   Please download the upper computer installation program from [RUBIK LINK V2.0](https://store.tmotor.com/product/rubik-link-v2.html) and configure all motors to MIT mode.
*   [AK70-10 Product Page](https://store.tmotor.com/product/ak70-10-dynamical-modular.html)
*   [AK Serials Manual](https://store.tmotor.com/images/file/202208/251661393360838805.pdf)

**Motor Order and Link/Joint Names:**
*   Motor Order: FR(1-3) --> FL(4-6) --> BR(7-9) --> BL(10-12) / Proximal --> Distal
*   Link Names: `*_scapula_link` --> `*_thigh_link` --> `*_calf_link`
*   Joint Names: `*_collar` --> `*_hip` --> `*_knee`

## 3. Software Dependencies

### 3.1 ROS 2 Jazzy Dependencies

The project is built on ROS 2 Jazzy. Key ROS 2 packages and their roles are:

*   **ament_cmake**: Core CMake build system for ROS 2 packages.
*   **rosidl_default_generators**: Generates C, C++, and Python code from `.msg`, `.srv`, and `.action` files.
*   **rosidl_default_runtime**: Provides runtime support for generated ROS IDL interfaces.
*   **std_msgs**: Standard message definitions (e.g., `String`, `Int32`).
*   **robot_state_publisher**: Publishes the robot's state to `tf2` based on URDF.
*   **rviz**: 3D visualization tool for ROS 2.
*   **joint_state_publisher_gui**: A GUI tool to manually publish joint states for URDF models.
*   **gazebo_ros_pkgs**: Integration with Gazebo simulator (for ROS 2, typically Gazebo Garden/Ignition).
*   **mevius_msgs**: Custom message package (inferred from `mevius_massage2` in `package.xml`).

Additional ROS-related dependencies:

*   [urdfdom-py](https://github.com/ros/urdf_parser_py): Python bindings for URDF parsing.
*   **realsense-ros**: For Intel RealSense cameras. **Note:** The `README.md` refers to a `ros1-legacy` branch. This needs to be updated to a ROS 2 compatible version or replaced with a ROS 2 native driver.
*   **spacenav_node**: For SpaceNavigator devices. **Note:** The `README.md` refers to a ROS 1 package. A ROS 2 equivalent or alternative is required.
*   **rqt_virtual_joystick**: A virtual joystick GUI. **Note:** The `README.md` refers to a ROS 1 package. A ROS 2 equivalent or alternative is required.

### 3.2 Python Package Dependencies

*   `bitstring`
*   `pytorch`
*   `scipy`
*   `mujoco`
*   `mujoco-python-viewer`

### 3.3 External Repositories / Libraries

*   [legged_gym](https://github.com/leggedrobotics/legged_gym)
    *   Specific fork/branch `haraduka/mevius` is used for reinforcement learning.
*   `mevius/tmotor_lib/tmotor_lib.py`: Originally from [mini-cheetah-tmotor-python-can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can)
*   `mevius/mevius_utils/legged_gym_math/isaacgym_torch_utils/isaacgym_torch_utils.py`: Originally from [IsaacGym](https://developer.nvidia.com/isaac-gym)
*   `mevius/mevius_utils/legged_gym_math/legged_gym_math.py`: Originally from [LeggedGym](https://github.com/leggedrobotics/legged_gym)

## 4. Build Instructions

This project uses `ament_python` for its Python packages and `ament_cmake` for C++ components (if any).

```bash
mkdir -p ~/mevius_ws/src
cd ~/mevius_ws/src
git clone git@github.com:haraduka/mevius.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 5. Reinforcement Learning

The project includes reinforcement learning policies. The current policy is described as simple and not very stable, with improvements planned for upload.

### Training Process (within `legged_gym` environment):

1.  Add `haraduka/legged_gym` as a remote and checkout the `haraduka/mevius` branch.
2.  **First Phase Training:**
    ```bash
    python3 legged_gym/scripts/train.py --task mevius
    ```
3.  **Second Phase Training:**
    *   Modify parameters in `legged_gym/envs/mevius/mevius_config.py` from "first" to "second" (specifically `commands.ranges` and `domain_rand.friction_range`).
    ```bash
    python3 legged_gym/scripts/train.py --task mevius
    ```
4.  **Playing the Policy:**
    ```bash
    python3 legged_gym/scripts/play.py --task mevius --load_run ( log_dir )
    ```
5.  **Exporting Policy:**
    ```bash
    cp logs/flat_mevius/( log_dir )/exported/policies/policy_1.pt ../mevius/models/policy.pt
    ```

## 6. Usage Instructions

### 6.1 Basic Utility Test

```bash
python3 scripts/mevius_utils.py
```

### 6.2 Motor Test

1.  CAN Interface Setup:
    ```bash
    ./bin/can_setup.sh
    ```
2.  Motor Control Test:
    ```bash
    python scripts/tmotor_test.py --ids 1 --task sense
    ```

### 6.3 Real Robot Operation (ROS 2 Jazzy)

*   **On MEVIUS PC:**
    ```bash
    # Start ROS 2 daemon if not already running
    ros2 daemon start 
    # Launch RealSense camera (requires ROS 2 compatible realsense-ros package)
    ros2 launch realsense2_camera rs_t265.launch 
    ./bin/can_setup.sh
    python3 scripts/mevius_main.py
    ```
*   **On Local PC:**
    ```bash
    # Launch SpaceNavigator node (requires ROS 2 compatible spacenav_node package)
    ros2 launch spacenav_node classic.launch
    ```
    *   **Control:** Left Click: SITDOWN/STANDUP, Right Click: STANDUP/WALK

### 6.4 Simulation

*   **MuJoCo Simulation:**
    ```bash
    python3 scripts/mevius_main.py --sim
    ```
    *   Press 'D' to turn off every-step rendering in the MuJoCo viewer.

## 7. Acknowledgements

*   `mevius/tmotor_lib/tmotor_lib.py`: Adapted from [mini-cheetah-tmotor-python-can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can)
*   `mevius/mevius_utils/legged_gym_math/isaacgym_torch_utils/isaacgym_torch_utils.py`: Adapted from [IsaacGym](https://developer.nvidia.com/isaac-gym)
*   `mevius/mevius_utils/legged_gym_math/legged_gym_math.py`: Adapted from [LeggedGym](https://github.com/leggedrobotics/legged_gym)

## 8. Citation

```
@inproceedings{kawaharazuka2024mevius,
  author={K. Kawaharazuka and S. Inoue and T. Suzuki and S. Yuzai and S. Sawaguchi and K. Okada and M. Inaba},
  title={{MEVIUS: A Quadruped Robot Easily Constructed through E-Commerce with Sheet Metal Welding and Machining}},
  booktitle={Proceedings of the 2024 IEEE-RAS International Conference on Humanoid Robots},
  year=2024,
}
```
