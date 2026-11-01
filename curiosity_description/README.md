# curiosity_description

ROS 2 description of the NASA **Curiosity (MSL)** rover, ready for **RViz**, **Gazebo (gz sim Harmonic)**,
**Unity** and any URDF-based simulator.

- Textured visual meshes (`.dae`), convex-hull collision meshes (`.stl`) and inertia for every part (899 kg total)
- Rocker-bogie suspension with the differential, 4 steered corner wheels, 6 driven wheels
- Remote-sensing mast (deploy, pan, tilt), high-gain antenna, 5-DOF arm with turret, covers
- Cameras with optical frames, an RGB-D depth camera, IMU and optional lidar, all with Gazebo sensors
- A Mars test site: hills, craters, a ridge, ~800 small rocks, 29 boulders, Mars textures and sky
- `ros2_control` for Gazebo: wheel velocity, steering position, smooth arm / mast trajectories

## Layout

```
curiosity_description/
├── urdf/
│   ├── curiosity.urdf.xacro   entry point (arguments)
│   ├── curiosity.xacro        links, joints, inertia, collision
│   ├── sensors.xacro          camera / IMU / lidar frames and Gazebo sensors
│   └── gazebo.xacro           ros2_control, Gazebo plugins, wheel friction
├── meshes/visual/             <link>.dae + textures
├── meshes/collision/          <link>.stl
├── config/
│   ├── controllers.yaml       ros2_control controllers
│   ├── self_filter.yaml       removes the rover's own body from lidar / depth data
│   └── gz_bridge.yaml         Gazebo <-> ROS 2 topics
├── worlds/
│   ├── mars.sdf               Mars terrain (default): hills, craters, rocks, boulders, sky
│   ├── mars/                  terrain / rock / sky meshes and textures
│   └── flat.sdf               flat ground for precise driving tests
├── rviz/display.rviz
└── launch/
    ├── display.launch.xml     RViz + joint sliders
    └── gazebo.launch.xml      Gazebo + controllers + bridge (+ RViz)
```

## Run

```bash
colcon build && source install/setup.bash

ros2 launch curiosity_description display.launch.xml       # RViz, move every joint with sliders
ros2 launch curiosity_description gazebo.launch.xml        # Gazebo Harmonic on Mars + RViz
ros2 launch curiosity_description gazebo.launch.xml world:=$(ros2 pkg prefix curiosity_description)/share/curiosity_description/worlds/flat.sdf
```

Command the rover in Gazebo:

```bash
# steering [front_left, front_right, rear_left, rear_right] in rad
ros2 topic pub --once /steering_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.3, -0.3, -0.3]}"
# wheels [front_l, front_r, middle_l, middle_r, rear_l, rear_r] in rad/s  (1 rad/s = 0.243 m/s)
ros2 topic pub -r 10 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1, 1, 1, 1, 1, 1]}"
# arm / mast / HGA / covers: trajectory_msgs/JointTrajectory on /articulation_controller/joint_trajectory
```

Plain URDF for Unity or other tools: `xacro urdf/curiosity.urdf.xacro use_gazebo:=false > curiosity.urdf`

## xacro arguments

| argument | default | |
|---|---|---|
| `use_gazebo` | `true` | ros2_control and Gazebo plugins / sensors |
| `use_cameras` | `true` | Gazebo sensors for NavCam, MastCam and HazCam; `false` keeps the frames and runs about 4× faster without a GPU |
| `use_lidar` | `true` | 16-beam lidar on a post on the deck (not on the real rover) |
| `lidar_x`, `lidar_y`, `lidar_z` | `0.755 0 0.266` | lidar post position in `base_link` |
| `wheel_collision` | `sphere` | `sphere`, `cylinder` or `mesh` |
| `differential_coupling` | `true` | right rocker = −left rocker (the real differential) |

## Frames and joints

REP-103: x forward, y left, z up. `base_footprint` is on the ground at the turning centre; `base_link` is 0.878 m above it.
At q = 0 the rover is fully deployed and every link frame is parallel to `base_link`.

| group | joints |
|---|---|
| drive | `wheel_{front,middle,rear}_{left,right}_joint` (continuous, axis +y, radius 0.2426 m) |
| steering | `steer_{front,rear}_{left,right}_joint` (±1.57 rad) |
| suspension | `rocker_left_joint` (passive), `bogie_{left,right}_joint` (passive); `rocker_right_joint`, `differential_joint`, `diff_rod_{left,right}_joint` mimic the left rocker; `rocker_{front,rear}_fold_*_joint` stowage hinges |
| mast | `mast_deploy_joint`, `mast_azimuth_joint`, `mast_elevation_joint` |
| antenna | `hga_azimuth_joint`, `hga_elevation_joint` |
| arm | `arm_joint_1` … `arm_joint_5`, `mahli_cover_joint` |
| covers | `deck_cover_{1,2,3}_joint`, `front_hazcam_cover_joint`, `rear_hazcam_cover_joint` |

Sensor frames: `navcam_{left,right}` (stereo, 0.42 m baseline, 45°), `mastcam_{left,right}` (15° / 5.7°),
`front_hazcam`, `rear_hazcam_{left,right}` (120°), `mardi`, `mahli` (frames only), `depth_camera` (RGB-D, 69°,
0.1–20 m, on the mast head pitched 20° down), each as `<name>_link` and `<name>_optical_frame`, plus `imu_link`
and `lidar_link`.

## Gazebo topics

| topic | type |
|---|---|
| `/joint_states`, `/tf` | all joints from `joint_state_broadcaster` + `robot_state_publisher` |
| `/<camera>/image_raw`, `/<camera>/camera_info` | `navcam_left/right`, `mastcam_left/right`, `front_hazcam`, `rear_hazcam_left/right` (with `use_cameras:=true`) |
| `/depth_camera/image_raw`, `/depth_camera/depth/image_raw` | RGB and depth (32FC1, metres), optical frame |
| `/depth_camera/points`, `/depth_camera/camera_info` | point cloud for obstacle detection (frame `depth_camera_link`), calibration |
| `/lidar/points`, `/lidar/scan` | PointCloud2, LaserScan (raw, include hits on the rover) |
| `/lidar/points_filtered`, `/lidar/scan_filtered`, `/depth_camera/points_filtered` | same data with the rover removed; use these for mapping and obstacle avoidance |
| `/imu` | Imu |
| `/odom/ground_truth` | Odometry, ground truth for evaluating odometry / SLAM (no TF published) |
| `/clock` | simulation time |

## The Mars world

`worlds/mars.sdf` is a 100 × 100 m driveable site plus a horizon ring and a butterscotch sky, at Mars gravity
(3.721 m/s²). It has rolling hills, three craters, a low ridge, five fields of small rocks (about 800, with
collision), 29 boulders and 51 larger rocks in six mineral colours as obstacles, a dark dune field and six sand
patches. The rover spawns in a flat,
rock-free zone at the origin. Textures are tileable regolith and basalt; all of it is static.

RViz shows the robot, TF, lidar, the depth cloud, the IMU, and image panels for NavCam left, the depth image,
Front HazCam and MastCam left (the last two are off by default; tick them in Displays).

## Notes

- **Physics:** DART with Bullet collision detection (set in `worlds/mars.sdf`).
  - Wheels collide as spheres: one clean contact point under each hub gives exact rolling and turning.
  - DART ignores URDF mimic joints, so `gz_ros2_control` enforces the differential.
- **Arm:** in Gazebo the arm spawns stowed (`gz_ros2_control` initial values) so it stays out of the
  cameras' and lidar's view; in RViz q = 0 is the deployed pose.
- **IMU:** the mean is correct (3.72 m/s², gyros ±0.003 rad/s), but the accelerometer shows ±0.4 m/s² of contact
  jitter from the physics solver. Use an accelerometer covariance around 0.4 m/s² or low-pass it before fusion.
- **Performance:** the Mars world plus seven mission cameras, the RGB-D camera and the lidar need a GPU for near
  real time. Without one, Gazebo runs at about 0.09× (CPU-only Docker container); `use_cameras:=false` brings it to
  about 0.38× headless. The controller spawner has long timeouts so startup still succeeds.
- **Self filter:** the deck lidar sees the rover itself (over half of its valid points). `gazebo.launch.xml`
  starts `pcl_ros` CropBox filters and a `laser_filters` box filter (`self_filter:=true`, the default) that remove
  every point inside a box around the body, wheels, mast and stowed arm, measured in `base_footprint`. Tested:
  5,183 rover points removed from one lidar scan, all 3,893 ground / rock points kept. The box does not cover the
  deployed arm (it reaches x = 2.32 m); widen `max_x` in `config/self_filter.yaml` if you drive with it out.
- **MastCam-100:** simulated at 5.7°, the narrowest field of view SDFormat accepts (the real camera is 5.1°).
- **Limits and masses:** arm / mast limits and link masses are estimates. Check them against the MSL specs
  before planning motions.
- **3D model:** [Curiosity Rover (MSL)](https://science.nasa.gov/3d-resources/curiosity-rover-msl/) by NASA
  (NASA 3D Resources). Not affiliated with or endorsed by NASA or JPL.
- **Licence:** MIT © 2026 Azer Ben Ali for the description and simulation; the NASA model data is not covered,
  see [LICENSE](../LICENSE).
