#!/usr/bin/env python3
"""Test drive: drives the rover in an already running simulation and checks it moved as commanded.
Works on the Mars terrain and on flat ground. Times are simulated seconds.

    docker compose up -d gazebo
    docker compose exec gazebo python3 /ws/tests/test_drive.py
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

R = 0.243                                                   # wheel radius, m (1 rad/s = 0.243 m/s)
WHEELS = [f"wheel_{p}_{s}_joint" for p in ("front", "middle", "rear") for s in ("left", "right")]
TURN_STEER = [-0.81696, 0.81599, 0.81718, -0.81602]           # corner wheels tangent to the turn circle
TURN_WHEEL = [-1.26249, 1.26625, -0.97038, 0.97430, -1.26224, 1.26640]
ARC_RADIUS = 4.0                                              # m, turn centre to the left of the body centre
RES = []


def check(name, ok, detail=""):
    RES.append(bool(ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}", flush=True)


class Drive(Node):
    def __init__(self):
        super().__init__("test_drive", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.pose = self.js = None
        self.tilt = 0.0
        self.create_subscription(Odometry, "odom/ground_truth", self.on_odom, 10)
        self.create_subscription(JointState, "joint_states", lambda m: setattr(self, "js", m), 10)
        self.steer = self.create_publisher(Float64MultiArray, "steering_controller/commands", 10)
        self.wheel = self.create_publisher(Float64MultiArray, "wheel_velocity_controller/commands", 10)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def on_odom(self, m):
        self.pose = m.pose.pose
        _, _, _, _, roll, pitch = self.xyyaw()
        self.tilt = max(self.tilt, abs(roll), abs(pitch))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def xyyaw(self):
        p, r = self.pose.position, self.pose.orientation
        yaw = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z))
        roll = math.atan2(2 * (r.w * r.x + r.y * r.z), 1 - 2 * (r.x * r.x + r.y * r.y))
        pitch = math.asin(max(-1, min(1, 2 * (r.w * r.y - r.z * r.x))))
        return p.x, p.y, p.z, yaw, roll, pitch

    def wheel_xy(self, w):
        p = self.buf.lookup_transform("base_footprint", w.replace("_joint", "_link"), Time()).transform.translation
        return p.x, p.y

    def wheels(self):
        q = dict(zip(self.js.name, self.js.position))
        return [q[w] for w in WHEELS]

    def hold(self, sec, steer, wheel):
        t0 = self.now()
        while self.now() - t0 < sec:
            self.steer.publish(Float64MultiArray(data=[float(x) for x in steer]))
            self.wheel.publish(Float64MultiArray(data=[float(x) for x in wheel]))
            rclpy.spin_once(self, timeout_sec=0.05)

    def leg(self, sec, steer, wheel):
        """Steer first (wheels stopped), then drive. Returns forward, lateral, yaw change and wheel travel."""
        self.hold(2.0, steer, [0.0] * 6)
        x0, y0, _, yaw0, _, _ = self.xyyaw()
        q0 = self.wheels()
        self.hold(sec, steer, wheel)
        x1, y1, _, yaw1, _, _ = self.xyyaw()
        q1 = self.wheels()
        dx, dy = x1 - x0, y1 - y0
        fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        return fwd, lat, math.remainder(yaw1 - yaw0, 2 * math.pi), [b - a for a, b in zip(q0, q1)]


rclpy.init()
d = Drive()
print("waiting for the simulation (start it with: docker compose up -d gazebo)...", flush=True)
t0 = time.time()
while (d.pose is None or d.js is None or d.now() == 0) and time.time() - t0 < 120:
    rclpy.spin_once(d, timeout_sec=0.1)
if d.pose is None or d.js is None:
    print("no /odom/ground_truth or /joint_states: is the simulation running?")
    sys.exit(2)
wall0, sim0 = time.time(), d.now()
x, y, _, yaw, _, _ = d.xyyaw()
print(f"start at x {x:+.2f} m, y {y:+.2f} m, heading {math.degrees(yaw):+.1f} deg\n", flush=True)

print("== drive forward 1 rad/s for 6 s")
fwd, lat, dyaw, dq = d.leg(6.0, [0.0] * 4, [1.0] * 6)
rolled = sum(dq) / 6 * R
check("rover drives forward", fwd > 0.8 * rolled > 0, f"body {fwd:.2f} m, wheels rolled {rolled:.2f} m")
check("keeps its heading", abs(dyaw) < math.radians(5), f"lateral {lat:+.2f} m, yaw {math.degrees(dyaw):+.1f} deg")

print("\n== reverse 1 rad/s for 3 s")
fwd, lat, dyaw, dq = d.leg(3.0, [0.0] * 4, [-1.0] * 6)
rolled = sum(dq) / 6 * R
check("rover backs up", fwd < 0.8 * rolled < 0, f"body {fwd:+.2f} m, wheels rolled {rolled:+.2f} m")

print("\n== turn on the spot (left) for 6 s")
fwd, lat, dyaw, dq = d.leg(6.0, TURN_STEER, TURN_WHEEL)
ymid = abs(d.buf.lookup_transform("base_footprint", "wheel_middle_left_link", Time()).transform.translation.y)
want = (dq[3] - dq[2]) * R / (2 * ymid)                      # from the unsteered middle wheels
check("turns left by the wheel travel", dyaw > 0 and abs(dyaw - want) < 0.25 * abs(want),
      f"{math.degrees(dyaw):+.1f} deg vs {math.degrees(want):+.1f} deg from the wheels")
check("stays on the spot", math.hypot(fwd, lat) < 0.25, f"centre moved {math.hypot(fwd, lat) * 100:.0f} cm")

print(f"\n== arc left on a {ARC_RADIUS:.0f} m radius at 0.24 m/s for 6 s")
xy = {w: d.wheel_xy(w) for w in WHEELS}
corners = [WHEELS[i] for i in (0, 1, 4, 5)]
arc_steer = [math.atan2(xy[w][0], ARC_RADIUS - xy[w][1]) for w in corners]   # every wheel tangent to its circle
arc_wheel = [1.0 / ARC_RADIUS * math.hypot(xy[w][0], ARC_RADIUS - xy[w][1]) for w in WHEELS]
fwd, lat, dyaw, dq = d.leg(6.0, arc_steer, arc_wheel)
want = (dq[3] - dq[2]) * R / (2 * ymid)
check("turns left by the wheel travel", dyaw > 0 and abs(dyaw - want) < 0.25 * abs(want),
      f"{math.degrees(dyaw):+.1f} deg vs {math.degrees(want):+.1f} deg from the wheels")
check("follows the circle", fwd > 0.5 and abs(lat - ARC_RADIUS * (1 - math.cos(dyaw))) < 0.1,
      f"forward {fwd:.2f} m, left {lat:+.2f} m (circle: {ARC_RADIUS * (1 - math.cos(dyaw)):+.2f} m)")

print("\n== stop")
d.hold(2.0, [0.0] * 4, [0.0] * 6)
x0, y0, _, _, _, _ = d.xyyaw()
d.hold(3.0, [0.0] * 4, [0.0] * 6)
x1, y1, _, _, _, _ = d.xyyaw()
check("holds still after stopping", math.hypot(x1 - x0, y1 - y0) < 0.02, f"drift {math.hypot(x1 - x0, y1 - y0) * 1000:.0f} mm in 3 s")
check("stayed upright the whole drive", d.tilt < math.radians(25), f"max roll/pitch {math.degrees(d.tilt):.1f} deg")

x, y, _, yaw, _, _ = d.xyyaw()
sim, wall = d.now() - sim0, time.time() - wall0
print(f"\nend at x {x:+.2f} m, y {y:+.2f} m, heading {math.degrees(yaw):+.1f} deg "
      f"({sim:.0f} s simulated in {wall:.0f} s, {100 * sim / wall:.0f} % real time)")
print(f"{sum(RES)}/{len(RES)} checks passed")
sys.exit(0 if all(RES) else 1)
