<div align="center">

<img src="docs/media/blender_rover.gif" alt="Curiosity rover: Blender animation of the deploy, drive and turn sequence, driving over the CR and radiation logos painted on the ground" width="100%"/>

# Curiosity Rover · ROS 2 Description

**NASA's Curiosity rover as a ready-to-run ROS 2 robot, in one Docker image for macOS, Windows and Linux.**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![Docker](https://img.shields.io/badge/Docker-amd64%20%7C%20arm64-2496ED?logo=docker&logoColor=white)
![License: MIT](https://img.shields.io/badge/License-MIT-green)

[Demo video](docs/media/demo.mp4) ·
[3D model](docs/model/curiosity_rover.stl) ·
[Reference](docs/REFERENCE.md) ·
[Package docs](curiosity_description/README.md)

</div>

<br/>

<img src="docs/media/demo.gif" alt="Four panels: RViz, Gazebo, RGB camera and depth camera while the rover drives" width="100%"/>

## Quick start

```bash
git clone <this repo> curiosity_rover
cd curiosity_rover
docker compose build     # once: about 10 min, ~6.5 GB image
docker compose up demo
```

Open **<http://localhost:6080>**. The desktop runs in your browser, and the windows appear after 30–60 s without a GPU.

Other services: `gazebo`, `gazebo-flat` and `rviz`. To drive the rover, open a second terminal with `docker compose exec gazebo bash` and run `python3 /ws/examples/drive_demo.py`.

## What's inside

- **Model:** 62 links, 61 joints (37 movable), textured visuals and collision meshes
- **Motion:** rocker-bogie suspension, 4 steered and 6 driven wheels, mast, antenna, 5-DOF arm
- **Sensors:** seven mission cameras, RGB-D camera, IMU, optional lidar
- **Simulation:** Gazebo Harmonic, `ros2_control` and a 100 × 100 m Mars world

Without a GPU, simulation runs slower than real time. See the [reference](docs/REFERENCE.md) for GPU setup, topics, joints, native install and tests.

Next up: navigation, SLAM, localisation and arm motion planning, in separate packages.

## Credits and license

3D model: [Curiosity Rover (MSL)](https://science.nasa.gov/3d-resources/curiosity-rover-msl/) by NASA, used under the [NASA media guidelines](https://www.nasa.gov/nasa-brand-center/images-and-media/). Not affiliated with or endorsed by NASA or JPL.

[MIT](LICENSE) © 2026 Azer Ben Ali. The original NASA 3D model is not covered by the MIT licence; see [LICENSE](LICENSE).
