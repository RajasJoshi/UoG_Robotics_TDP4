# ros_gz_project_template
A template project integrating ROS 2 and Gazebo simulator.

## Included packages

* `nao_description` - holds the sdf description of the simulated system and any other assets.

* `nao_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `nao_application` - holds ros2 specific code and configurations.

* `nao_bringup` - holds launch files and high level utilities.


## Install
### Requirements

1. Choose a ROS and Gazebo combination  https://gazebosim.org/docs/harmonic/ros_installation
   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

    ```bash
    export GZ_VERSION=garden
    ```

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

### Use as template
Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

   ```bash
   mkdir -p ~/template_ws/src
   cd ~/template_ws/src
   wget https://raw.githubusercontent.com/gazebosim/ros_gz_project_template/main/template_workspace.yaml
   vcs import < template_workspace.yaml
   ```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/<ROS_DISTRO>/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch nao_bringup diff_drive.launch.py
    ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
https://github.com/AbrahamZX/RBE521_Gait/tree/main
https://github.com/robogeekcanada/noetic_robots/tree/main

```
/clock
/gazebo/resource_paths
/gui/camera/pose
/stats
/world/nao_world/clock
/world/nao_world/dynamic_pose/info
/world/nao_world/pose/info
/world/nao_world/scene/deletion
/world/nao_world/scene/info
/world/nao_world/state
/world/nao_world/stats
/model/Nao/joint/HeadPitch/0/cmd_pos
/model/Nao/joint/HeadYaw/0/cmd_pos
/model/Nao/joint/LAnklePitch/0/cmd_pos
/model/Nao/joint/LAnkleRoll/0/cmd_pos
/model/Nao/joint/LElbowRoll/0/cmd_pos
/model/Nao/joint/LElbowYaw/0/cmd_pos
/model/Nao/joint/LHipPitch/0/cmd_pos
/model/Nao/joint/LHipRoll/0/cmd_pos
/model/Nao/joint/LHipYawPitch/0/cmd_pos
/model/Nao/joint/LKneePitch/0/cmd_pos
/model/Nao/joint/LShoulderPitch/0/cmd_pos
/model/Nao/joint/LShoulderRoll/0/cmd_pos
/model/Nao/joint/LWristYaw/0/cmd_pos
/model/Nao/joint/RAnklePitch/0/cmd_pos
/model/Nao/joint/RAnkleRoll/0/cmd_pos
/model/Nao/joint/RElbowRoll/0/cmd_pos
/model/Nao/joint/RElbowYaw/0/cmd_pos
/model/Nao/joint/RHipPitch/0/cmd_pos
/model/Nao/joint/RHipRoll/0/cmd_pos
/model/Nao/joint/RHipYawPitch/0/cmd_pos
/model/Nao/joint/RKneePitch/0/cmd_pos
/model/Nao/joint/RShoulderPitch/0/cmd_pos
/model/Nao/joint/RShoulderRoll/0/cmd_pos
/model/Nao/joint/RWristYaw/0/cmd_pos
/world/nao_world/light_config

```