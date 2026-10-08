# SO-ARM Gazebo pick and place

A self-contained ROS 2 Humble workspace for the **SO100** model in this repository. It mounts the arm on a 0.60 m table, picks up an orange 20 mm cube, moves it to the green target, releases it, and checks the cube's final position.

For the separate joint-slider simulation with live transform matrices, coordinate frames, tool trajectory and Jacobian visualization, see [the forward-kinematics learning guide](FK_LEARNING.md). Launch it with `ros2 launch so_arm_gazebo fk_learning.launch.py` after sourcing the workspace.

The robot geometry and kinematic joint definitions come from `../Simulation/SO100/so100.urdf`; the mesh files are copied into the package, so the installed simulation does not depend on a path outside this workspace. The original model is unchanged.

The target system is **Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11**. This implementation uses Humble's `gazebo_ros2_control` integration. Gazebo Classic reached end of life in January 2025; this is a compatibility choice for the requested Humble environment. Modern Gazebo uses a different simulator integration. See the [official Humble integration documentation](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html).

## Install dependencies

Start with ROS 2 Humble already installed and its apt repository configured. On Ubuntu 22.04:

```bash
sudo apt update
sudo apt install \
  gazebo libgazebo-dev \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-xacro \
  python3-colcon-common-extensions python3-rosdep python3-yaml python3-pytest
```

Resolve the package's declared dependencies as well:

```bash
source /opt/ros/humble/setup.bash
cd /data/SO-ARM100/so-arm_ws
# Run sudo rosdep init once only if rosdep has never been initialized.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

Adjust the workspace path if the repository is in another location. The workspace does not require downloading external Gazebo models; its world defines its own ground, sun, table, cube, and target.

## Build and run

Use a terminal with no active Conda environment. Humble's Python packages are built for the Ubuntu system Python. The explicit interpreter below also prevents Colcon/CMake from selecting a Conda Python:

```bash
source /opt/ros/humble/setup.bash
cd /data/SO-ARM100/so-arm_ws
/usr/bin/python3 /usr/bin/colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch so_arm_gazebo simulation.launch.py
```

One automatic cycle runs after the robot has spawned and all three controllers are active. Startup uses process results, service availability, and controller actions rather than fixed startup delays. A spawn/controller/demo failure reports the error and shuts down the launch. Successful completion leaves Gazebo open for inspection. Close the GUI or press `Ctrl+C` to stop.

Run the same scene without a GUI:

```bash
ros2 launch so_arm_gazebo simulation.launch.py gui:=false
```

Start the robot and controllers without the automatic cycle:

```bash
ros2 launch so_arm_gazebo simulation.launch.py demo:=false
```

In a second terminal, source both setup files and run the cycle manually:

```bash
source /opt/ros/humble/setup.bash
source /data/SO-ARM100/so-arm_ws/install/setup.bash
ros2 run so_arm_gazebo pick_and_place --ros-args \
  --params-file "$(ros2 pkg prefix --share so_arm_gazebo)/config/demo.yaml" \
  -p use_sim_time:=true
```

Restart the launch to restore the cube to its source position before another cycle. The demo expects the cube at that source position.

## Scene and control interface

All distances are metres and all revolute joint positions are **radians**, including the gripper. These values are URDF joint angles; they are not LeRobot motor percentages or calibrated hardware commands.

| Item | Configuration |
| --- | --- |
| Robot model name | `so100` |
| Base position in world | `(0, 0, 0.60)`; fixed to the world in the xacro |
| Table top | World `z = 0.60` |
| Cube | `pick_cube`, link `link`; side `0.020`, mass `0.020 kg` |
| Initial cube centre | `(0, -0.22, 0.61)` |
| Destination cube centre | `(0.11, -0.18, 0.61)` |
| Arm action | `/arm_controller/follow_joint_trajectory` |
| Gripper action | `/gripper_controller/follow_joint_trajectory` |
| Joint states | `/joint_states` |
| Object pose query | `/gazebo/get_entity_state` |
| Assisted grasp service | `/grasp/set_attached`, `std_srvs/srv/SetBool` |

The arm action joint order is `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`. The gripper action contains the single `gripper` joint. Both use `control_msgs/action/FollowJointTrajectory`. For example, when the automatic demo is disabled, open the jaw with:

```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['gripper'], points: [{positions: [0.55], time_from_start: {sec: 1}}]}}"
```

Check the controllers and object pose:

```bash
ros2 control list_controllers
ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
  "{name: 'pick_cube', reference_frame: 'world'}"
```

The broadcaster, arm controller, and gripper controller should all be `active`. Keep Gazebo unpaused while executing motions or calling the grasp service.

## How the demonstration works

The demo moves through configured approach, pick, lift, transfer, place, and retreat joint targets. It opens the jaw, reaches the cube, closes the jaw, attaches, lifts, transfers, opens while held, releases, and retreats. It checks the cube during the lift and after settling on the table. `tool0` is the added tool centre point, 0.100 m along the gripper's local negative Y axis.

The grasp is deliberately assisted: the package's local Gazebo model plugin creates a temporary physical fixed joint when the cube is near the tool centre and the jaw is closed. The cube is held at its current relative pose; it is not teleported onto a predefined path. Both jaw visuals have no collision geometry, so their motion cannot push the cube before attachment. The arm's collision meshes remain active. Cube collision is disabled while held, then restored on release, when gravity and table contact determine its final pose. You can inspect the plugin in `src/so_arm_gazebo/src/grasp_plugin.cpp`.

This is a small simulation example with simplified collision bodies and simulated position control. It does not include MoveIt, obstacle-aware planning, realistic motor/force control, or validation of a friction-only grasp. It does not send commands to real hardware.

## Customize

| File | Purpose |
| --- | --- |
| `src/so_arm_gazebo/worlds/pick_place.world` | Table, cube, target and physics |
| `src/so_arm_gazebo/urdf/so100.urdf.xacro` | Robot, mounting, collision bodies, ros2_control and grasp plugin |
| `src/so_arm_gazebo/config/controllers.yaml` | Controller interfaces and trajectory tolerances |
| `src/so_arm_gazebo/config/demo.yaml` | Waypoints, timing, object targets and verification tolerances |
| `src/so_arm_gazebo/launch/simulation.launch.py` | Launch orchestration |

If you move the source or destination, update both the world and demo configuration and choose reachable joint waypoints. Changing the cube size also requires reviewing jaw clearance and tool-centre geometry. Rebuild after changing code or package metadata, and restart Gazebo after changing the world or xacro.

## Troubleshooting

- **`Package 'so_arm_gazebo' not found`:** source `/opt/ros/humble/setup.bash` and this workspace's `install/setup.bash` in that terminal.
- **Python import or CMake Python errors:** deactivate Conda, use `/usr/bin/python3` for Colcon, and rebuild with the system Python options shown above.
- **Plugin library cannot be loaded:** build the workspace and source its install setup. The package exports its plugin directory, and the launch adds the package's `lib` directory to `GAZEBO_PLUGIN_PATH`.
- **Controller timeout:** check earlier Gazebo logs for a URDF or `gazebo_ros2_control` error. Do not pause physics during startup.
- **Grasp rejected:** check the cube's pose, jaw angle, and configured waypoints. The plugin rejects attachment when the jaw is open or the cube is outside its tool-centre radius.
- **GUI fails on a remote machine:** use `gui:=false` for a headless simulation. The GUI needs a working graphics/display session.

## Validation

Verified on this machine with Ubuntu 22.04, ROS 2 Humble, and Gazebo Classic 11.10.2: the Colcon build passed, all 14 kinematics checks passed, and the complete headless pick-and-place cycle passed. The final cube centre was `(0.1100, -0.1800, 0.6099)` m, with zero reported linear speed. The demo and Gazebo shut down cleanly. GUI rendering was not exercised in this check.

Run the kinematic checks and a finite headless integration check after building:

```bash
source /opt/ros/humble/setup.bash
cd /data/SO-ARM100/so-arm_ws
source install/setup.bash
/usr/bin/python3 /usr/bin/colcon test --event-handlers console_direct+
/usr/bin/python3 /usr/bin/colcon test-result --verbose
/usr/bin/python3 scripts/smoke_test.py
```

The headless check reports success only after Gazebo verifies the cube's lift, destination, and rest state, then stops its own launch processes. Its log is `log/smoke_test/simulation.log`. By default it uses ROS domain 73 and Gazebo port 11375 to separate it from other simulations; existing environment settings take precedence.
