# Tests

| file | checks | run |
|---|---|---|
| `test_gazebo.py` | starts the simulation headless on flat ground: 22 checks (below) | `docker compose run --rm test` |
| `test_drive.py` | drives the rover in a running simulation against ground-truth odometry: 9 checks | `docker compose exec gazebo python3 /ws/tests/test_drive.py` |
| `test_self_filter.py` | in a running simulation, no lidar or depth point is left on the rover and the environment is kept: 3 checks | `docker compose exec gazebo python3 /ws/tests/test_self_filter.py` |

`test_gazebo.py`:

- standing still: no drift, body level, no joint chatter, all 37 joints published
- steering reaches commanded angles, straight driving and slip, turn on the spot
- differential coupling
- arm stow move and hold against gravity
- camera images, `camera_info`, stereo pair, lidar, IMU (Mars gravity), TF, depth image and depth cloud

`test_drive.py`:

- forward and reverse travel match the wheel travel, heading held
- turn on the spot and a 4 m radius arc turn by what the wheels predict, the arc follows the circle
- holds still after stopping, stays upright
