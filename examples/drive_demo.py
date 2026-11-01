"""Scripted drive, 36 s of simulated time (works at any real-time factor):
look around with the mast, drive forward, turn on the spot, arc left on a 4 m radius, stop.
Steering angles and wheel speeds come from the wheel positions, so every wheel rolls without scrubbing.

    docker compose exec gazebo python3 /ws/examples/drive_demo.py
"""
import math

import rclpy
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

R = 0.2426                                                     # wheel radius, m
XY = {                                                         # wheel centres in base_footprint, m
    "front_left": (1.129, 1.060), "front_right": (1.129, -1.060),
    "middle_left": (-0.053, 1.189), "middle_right": (-0.053, -1.189),
    "rear_left": (-1.127, 1.058), "rear_right": (-1.127, -1.058),
}
WHEELS = ["front_left", "front_right", "middle_left", "middle_right", "rear_left", "rear_right"]
STEERED = ["front_left", "front_right", "rear_left", "rear_right"]
STOW = [-1.571, -0.262, -1.222, 1.484, -0.559]                 # arm_joint_1..5
MAST = [(4, 0.6, 0.3), (8, 0.0, 0.1), (26, 0.0, 0.1), (30, 0.45, 0.15), (36, 0.0, 0.1)]  # t, pan, tilt


def turn(w, rc):
    """Steering angles and wheel speeds (rad/s) to turn at w rad/s about the point (0, rc) in base_footprint."""
    steer, speed = {}, {}
    for n, (x, y) in XY.items():
        vx, vy = -w * (y - rc), w * x                          # wheel velocity = w x (wheel - centre)
        if n in STEERED:
            a, s = math.atan2(vy, vx), math.hypot(vx, vy) / R
            if abs(a) > math.pi / 2:                           # steer the short way and roll backwards
                a, s = a - math.copysign(math.pi, a), -s
            steer[n], speed[n] = a, s
        else:
            speed[n] = vx / R                                  # middle wheels are not steered
    return [steer[n] for n in STEERED], [speed[n] for n in WHEELS]


def main():
    rclpy.init()
    n = rclpy.create_node("drive_demo", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    ps = n.create_publisher(Float64MultiArray, "steering_controller/commands", 10)
    pw = n.create_publisher(Float64MultiArray, "wheel_velocity_controller/commands", 10)
    pt = n.create_publisher(JointTrajectory, "articulation_controller/joint_trajectory", 10)
    now = lambda: n.get_clock().now().nanoseconds * 1e-9        # noqa: E731
    while now() == 0 or min(p.get_subscription_count() for p in (ps, pw, pt)) == 0:
        rclpy.spin_once(n, timeout_sec=0.1)

    def hold(sec, steer, wheel):
        t0 = now()
        while now() - t0 < sec:
            ps.publish(Float64MultiArray(data=[float(v) for v in steer]))
            pw.publish(Float64MultiArray(data=[float(v) for v in wheel]))
            rclpy.spin_once(n, timeout_sec=0.05)

    tr = JointTrajectory(joint_names=[f"arm_joint_{i}" for i in range(1, 6)] + ["mast_azimuth_joint", "mast_elevation_joint"])
    tr.points = [JointTrajectoryPoint(positions=STOW + [pan, tilt], time_from_start=Duration(sec=t)) for t, pan, tilt in MAST]
    pt.publish(tr)                                             # mast looks around while the rover drives

    straight = [0.0] * 4
    spin = turn(0.2, 0.0)                                      # 0.2 rad/s on the spot
    arc = turn(R / 4.0, 4.0)                                   # 4 m radius at 0.24 m/s
    hold(4, straight, [0.0] * 6)                               # 0-4 s   look left and down
    hold(8, straight, [1.0] * 6)                               # 4-12 s  forward 1.9 m
    hold(2, spin[0], [0.0] * 6)                                # 12-14 s steer for the spin
    hold(8, *spin)                                             # 14-22 s turn left ~90 deg
    hold(2, arc[0], [0.0] * 6)                                 # 22-24 s steer for the arc
    hold(10, *arc)                                             # 24-34 s arc left 2.4 m
    hold(2, straight, [0.0] * 6)                               # 34-36 s straighten and stop
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
