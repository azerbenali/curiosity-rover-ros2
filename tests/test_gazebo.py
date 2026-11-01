#!/usr/bin/env python3
"""Gazebo Harmonic acceptance test for curiosity_description (kept outside the package).
Starts `gazebo.launch.xml` headless, then checks physics, joints, steering, driving, arm and sensors.
usage: python3 test_gazebo.py [outdir for camera snapshots]"""
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rosgraph_msgs.msg import Clock

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp"
RES = []


def check(name, ok, detail=""):
    RES.append((name, bool(ok)))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}", flush=True)


# ---------------------------------------------------------------- start the simulation
_share = subprocess.run(["ros2", "pkg", "prefix", "curiosity_description"], capture_output=True, text=True).stdout.strip()
launch = subprocess.Popen(["ros2", "launch", "curiosity_description", "gazebo.launch.xml",
                           "gz_args:=-r -s --headless-rendering", "rviz:=false",
                           f"world:={_share}/share/curiosity_description/worlds/flat.sdf"],
                          stdout=open(os.path.join(OUT, "gz_launch.log"), "w"), stderr=subprocess.STDOUT,
                          start_new_session=True)

urdf = subprocess.run(["xacro", os.path.join(subprocess.run(["ros2", "pkg", "prefix", "curiosity_description"],
                       capture_output=True, text=True).stdout.strip(), "share/curiosity_description/urdf/curiosity.urdf.xacro")],
                      capture_output=True, text=True, check=True).stdout
J = {}
for j in ET.fromstring(urdf).findall("joint"):          # top-level only (not ros2_control)
    J[j.find("child").get("link")] = (j.get("name"), j.find("parent").get("link"),
                                       np.array(j.find("origin").get("xyz").split(), float))


def pos0(link):                                   # link origin in base_footprint at q = 0
    p = np.zeros(3)
    while link in J:
        p += J[link][2]
        link = J[link][1]
    return p


WHEELS = [f"wheel_{p}_{s}" for p in ("front", "middle", "rear") for s in ("left", "right")]
STEER = [f"steer_{p}_{s}" for p in ("front", "rear") for s in ("left", "right")]
WP = {w: pos0(w + "_link") for w in WHEELS}
R = float(np.mean([WP[w][2] for w in WHEELS]))


class T(Node):
    def __init__(self):
        super().__init__("gz_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.js, self.pose, self.img, self.info, self.imu, self.pc = None, None, {}, {}, None, None
        self.depth = self.dpc = None
        self.create_subscription(JointState, "joint_states", lambda m: setattr(self, "js", m), 10)
        self.create_subscription(Odometry, "odom/ground_truth", lambda m: setattr(self, "pose", m.pose.pose), 10)
        for c in ("navcam_left", "navcam_right", "front_hazcam", "mastcam_left"):
            self.create_subscription(Image, f"{c}/image_raw", lambda m, c=c: self.img.__setitem__(c, m), 2)
            self.create_subscription(CameraInfo, f"{c}/camera_info", lambda m, c=c: self.info.__setitem__(c, m), 2)
        self.imus = []
        self.create_subscription(Imu, "imu", lambda m: (setattr(self, "imu", m), self.imus.append(m.linear_acceleration.z)), 10)
        self.create_subscription(PointCloud2, "lidar/points", lambda m: setattr(self, "pc", m), 2)
        self.create_subscription(Image, "depth_camera/depth/image_raw", lambda m: setattr(self, "depth", m), 2)
        self.create_subscription(PointCloud2, "depth_camera/points", lambda m: setattr(self, "dpc", m), 2)
        self.create_subscription(Clock, "clock", lambda m: None, 10)
        self.wheel = self.create_publisher(Float64MultiArray, "wheel_velocity_controller/commands", 10)
        self.steer = self.create_publisher(Float64MultiArray, "steering_controller/commands", 10)
        self.traj = self.create_publisher(JointTrajectory, "articulation_controller/joint_trajectory", 10)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_sim(self, sec, every=None):
        t0 = self.now()
        while self.now() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            if every:
                every()

    def q(self):
        return dict(zip(self.js.name, self.js.position))

    def xyyaw(self):
        p, r = self.pose.position, self.pose.orientation
        yaw = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z))
        roll = math.atan2(2 * (r.w * r.x + r.y * r.z), 1 - 2 * (r.x * r.x + r.y * r.y))
        pitch = math.asin(max(-1, min(1, 2 * (r.w * r.y - r.z * r.x))))
        return p.x, p.y, p.z, yaw, roll, pitch

    def cmd(self, steer, wheel):
        self.steer.publish(Float64MultiArray(data=[float(x) for x in steer]))
        self.wheel.publish(Float64MultiArray(data=[float(x) for x in wheel]))


rclpy.init()
t = T()
t0 = time.time()
while (t.js is None or t.pose is None or not t.img) and time.time() - t0 < 120:
    rclpy.spin_once(t, timeout_sec=0.1)
check("simulation up: joint_states, model pose and camera images arriving", t.js is not None and t.pose is not None and bool(t.img),
      f"{time.time() - t0:.0f} s to start")
t.wait_sim(3.0)

print("\n== standing still")
x0, y0, z0, yaw0, _, _ = t.xyyaw()
t.wait_sim(10.0, lambda: t.cmd([0] * 4, [0] * 6))
x1, y1, z1, yaw1, roll, pitch = t.xyyaw()
drift = math.hypot(x1 - x0, y1 - y0)
check("no drift while idle (10 s sim)", drift < 0.01 and abs(yaw1 - yaw0) < math.radians(0.2),
      f"moved {drift * 1000:.1f} mm, yaw {math.degrees(yaw1 - yaw0):+.3f} deg")
check("body level on flat ground", abs(roll) < math.radians(0.5) and abs(pitch) < math.radians(0.5),
      f"roll {math.degrees(roll):+.2f} deg, pitch {math.degrees(pitch):+.2f} deg")
vel = dict(zip(t.js.name, t.js.velocity))
check("all joints at rest (no chatter)", max(abs(v) for v in vel.values()) < 0.02,
      f"max |vel| {max(abs(v) for v in vel.values()):.4f} rad/s")
check("joint_states covers every movable joint", len(t.js.name) == 37, f"{len(t.js.name)} joints")

print("\n== steering")
target = [0.5, -0.5, 0.3, -0.3]
t.wait_sim(1.5, lambda: t.cmd(target, [0] * 6))
q = t.q()
err = max(abs(q[s + "_joint"] - v) for s, v in zip(STEER, target))
check("steering reaches commanded angles", err < 0.01, f"max error {err:.4f} rad")
t.wait_sim(1.5, lambda: t.cmd([0] * 4, [0] * 6))

print("\n== drive straight 1 rad/s for 8 s (sim)")
x0, y0, _, yaw0, _, _ = t.xyyaw()
q0 = t.q()
t.wait_sim(8.0, lambda: t.cmd([0] * 4, [1.0] * 6))
x1, y1, _, yaw1, _, _ = t.xyyaw()
q1 = t.q()
t.wait_sim(1.0, lambda: t.cmd([0] * 4, [0] * 6))
fwd = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
lat = -(x1 - x0) * math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)
rolled = np.mean([q1[w + "_joint"] - q0[w + "_joint"] for w in WHEELS]) * R
check("rover moves forward as far as the wheels roll", abs(fwd - rolled) < 0.05 * rolled,
      f"body {fwd:.3f} m, wheels {rolled:.3f} m (slip {100 * (rolled - fwd) / rolled:+.1f} %)")
check("drives straight", abs(lat) < 0.03 * fwd and abs(yaw1 - yaw0) < math.radians(2),
      f"lateral {lat * 1000:.0f} mm, yaw {math.degrees(yaw1 - yaw0):+.2f} deg")

print("\n== turn in place at 0.2 rad/s for 8 s (sim)")
w = 0.2
steer, wheel = [], []
for s in STEER:
    x, y = WP["wheel" + s[5:]][:2]
    d = math.atan2(w * x, -w * y)
    sp = math.hypot(w * x, w * y)
    if abs(d) > math.pi / 2:
        d -= math.copysign(math.pi, d); sp = -sp
    steer.append(d)
corner = {}
for s, d in zip(STEER, steer):
    x, y = WP["wheel" + s[5:]][:2]
    corner["wheel" + s[5:]] = ((-w * y) * math.cos(d) + (w * x) * math.sin(d)) / R
for wn in WHEELS:
    wheel.append(corner.get(wn, (-w * WP[wn][1]) / R))
t.wait_sim(2.0, lambda: t.cmd(steer, [0] * 6))           # stop - steer
x0, y0, _, yaw0, _, _ = t.xyyaw()
t.wait_sim(8.0, lambda: t.cmd(steer, wheel))            # go
x1, y1, _, yaw1, _, _ = t.xyyaw()
t.wait_sim(1.0, lambda: t.cmd(steer, [0] * 6))
dyaw = math.remainder(yaw1 - yaw0, 2 * math.pi)
check("turns at the commanded rate", abs(dyaw - w * 8) < 0.1 * w * 8, f"{math.degrees(dyaw):.1f} deg vs {math.degrees(w * 8):.1f}")
check("turns on the spot", math.hypot(x1 - x0, y1 - y0) < 0.08, f"centre moved {math.hypot(x1 - x0, y1 - y0) * 1000:.0f} mm")
t.wait_sim(2.0, lambda: t.cmd([0] * 4, [0] * 6))

print("\n== suspension: differential")
q = t.q()
L = q["rocker_left_joint"]
check("right rocker mirrors left, bar + rods follow (mimic enforced)",
      abs(q["rocker_right_joint"] + L) < 2e-3 and abs(q["differential_joint"] + 0.419101 * L) < 2e-3
      and abs(q["diff_rod_left_joint"] - 0.268539 * L) < 2e-3,
      f"left {L:+.4f} right {q['rocker_right_joint']:+.4f} bar {q['differential_joint']:+.4f}")

print("\n== arm: spawns stowed, smooth deploy move, holds against gravity")
arm = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
q = t.q()
start = [-1.571, -0.262, -1.222, 1.484, -0.559]
check("arm spawns stowed (gz_ros2_control initial values)", max(abs(q[a] - v) for a, v in zip(arm, start)) < 0.01,
      str([round(q[a], 3) for a in arm]))
stow = [0.0] * 5                                                  # deployed pose of the model
tr = JointTrajectory(joint_names=arm)
tr.points = [JointTrajectoryPoint(positions=stow, velocities=[0.0] * 5, time_from_start=Duration(sec=6))]
t.traj.publish(tr)
peak = 0.0
def track():
    global peak
    v = dict(zip(t.js.name, t.js.velocity)); peak = max(peak, max(abs(v[a]) for a in arm))
t.wait_sim(7.5, track)
q = t.q()
err = max(abs(q[a] - s) for a, s in zip(arm, stow))
check("arm reaches the deployed pose", err < 0.01, f"max error {err:.4f} rad, peak speed {peak:.2f} rad/s")
t.wait_sim(3.0)
q2 = t.q()
sag = max(abs(q2[a] - q[a]) for a in arm)
check("arm holds its pose (no sag)", sag < 0.002, f"moved {sag:.5f} rad in 3 s")

print("\n== sensors")
img = t.img.get("navcam_left")
check("navcam_left image", img is not None and img.width == 800 and img.height == 800
      and img.header.frame_id == "navcam_left_optical_frame", f"{img.width}x{img.height} {img.encoding} frame={img.header.frame_id}")
ci = t.info.get("navcam_left")
fx = ci.k[0]
check("camera_info matches the 45 deg lens", abs(fx - 400 / math.tan(0.7854 / 2)) < 2, f"fx = {fx:.1f} px")
check("stereo pair both streaming", "navcam_right" in t.img, "")
check("lidar point cloud", t.pc is not None and t.pc.width * t.pc.height > 1000 and t.pc.header.frame_id == "lidar_link",
      f"{t.pc.width * t.pc.height} points frame={t.pc.header.frame_id}")
t.imus.clear(); t.wait_sim(1.0)
az = np.array(t.imus)
check("IMU reads Mars gravity (1 s mean)", abs(az.mean() - 3.721) < 0.1 and t.imu.header.frame_id == "imu_link",
      f"az = {az.mean():.3f} m/s^2 (std {az.std():.2f}, {len(az)} samples) frame={t.imu.header.frame_id}")
try:
    tf = t.buf.lookup_transform("base_footprint", "navcam_left_optical_frame", Time())
    check("TF base_footprint -> navcam_left_optical_frame", tf.transform.translation.z > 1.9,
          f"z = {tf.transform.translation.z:.3f} m")
except Exception as e:                                          # noqa: BLE001
    check("TF base_footprint -> navcam_left_optical_frame", False, str(e)[:80])

d = t.depth
D = np.frombuffer(bytes(d.data), np.float32).reshape(d.height, d.width) if d else np.zeros((1, 1))
check("depth image (32FC1, metres, optical frame)", d is not None and d.encoding == "32FC1"
      and d.header.frame_id == "depth_camera_optical_frame" and np.isfinite(D).mean() > 0.5,
      f"{d.width}x{d.height} valid {100 * np.isfinite(D).mean():.0f}% frame={d.header.frame_id}")
from sensor_msgs_py import point_cloud2 as pc2
P = np.array([tuple(p)[:3] for p in pc2.read_points(t.dpc, field_names=("x", "y", "z"), skip_nans=True)])
P = P[np.isfinite(P).all(1)]
tf = t.buf.lookup_transform("base_footprint", t.dpc.header.frame_id, Time()).transform
qx, qy, qz, qw = tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
Rm = np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)], [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
               [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
B = P @ Rm.T + np.array([tf.translation.x, tf.translation.y, tf.translation.z])
g = B[(np.hypot(B[:, 0], B[:, 1]) > 4) & (np.hypot(B[:, 0], B[:, 1]) < 15)]
check("depth point cloud lands on the ground through TF", len(g) > 1000 and abs(np.median(g[:, 2])) < 0.03,
      f"{len(P)} points frame={t.dpc.header.frame_id}, ground z median {np.median(g[:, 2]):+.3f} m")

for c, m in t.img.items():                                       # snapshots for the user
    raw = os.path.join(OUT, f"{c}.rgb")
    open(raw, "wb").write(bytes(m.data))
    subprocess.run(["convert", "-size", f"{m.width}x{m.height}", "-depth", "8", f"rgb:{raw}", os.path.join(OUT, f"{c}.png")])
    os.remove(raw)

os.killpg(launch.pid, 15)
n_fail = sum(1 for _, ok in RES if not ok)
print(f"\n{len(RES) - n_fail}/{len(RES)} checks passed")
sys.exit(1 if n_fail else 0)
