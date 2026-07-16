# excavator_description

ROS 2 Jazzy description package for the excavator model. It provides the URDF,
meshes, `robot_state_publisher` launch file, manual joint-state slider, RViz
configuration, and an optional bucket-tip monitoring topic.

## Build and launch

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/excavator_ws
colcon build --symlink-install --packages-select excavator_description
source install/setup.bash
ros2 launch excavator_description display.launch.py
```

`display.launch.py` starts `robot_state_publisher` and the optional bucket-tip
pose publisher. It does not start RViz or publish `/joint_states`.

For manual validation, run one joint-state source in a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/excavator_ws/install/setup.bash
ros2 launch excavator_description slider.launch.py \
  swing:=0.0 boom:=0.0 arm:=0.0 bucket:=0.0
```

The expected joints are:

```text
swing_joint
boom_joint
arm_joint
bucket_joint
```

Do not run the slider together with `excavator_sensor_bridge`; both publish
`/joint_states`.

Start RViz separately when needed:

```bash
ros2 launch excavator_description rviz.launch.py
```

## TF ownership

The URDF and `robot_state_publisher` own this excavator TF chain:

```text
world
`-- base_link
    `-- swing_link
        `-- boom_link
            `-- arm_link
                `-- bucket_link
                    `-- bucket_tip
```

The package does not define the external LiDAR calibration. The perception
pipeline additionally requires a calibrated `world -> rslidar` transform,
published by exactly one source.

## Bucket-tip pose

By default, `display.launch.py` republishes the latest `world -> bucket_tip`
transform as:

```text
/bucket_tip_pose_world  # geometry_msgs/msg/PoseStamped
```

This topic is for visualization and diagnostics. Point-cloud filtering and dig
point selection must query TF using the source message timestamp; they must not
use this latest-value topic for synchronization.

```bash
ros2 topic echo /bucket_tip_pose_world
ros2 launch excavator_description display.launch.py \
  use_tip_pose_publisher:=false
```
