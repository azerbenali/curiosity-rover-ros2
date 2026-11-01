"""Check the rover self filter on a running gazebo.launch.xml: points inside the rover box are
removed, everything outside is kept. usage: python3 test_self_filter.py"""
import math, sys, time
import numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

BOX = dict(min_x=-1.68, max_x=1.47, min_y=-1.45, max_y=1.45, min_z=0.04, max_z=2.27)
rclpy.init()
n = rclpy.create_node("self_filter_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
buf = Buffer(); TransformListener(buf, n)
got = {}
for t in ("lidar/points", "lidar/points_filtered", "depth_camera/points", "depth_camera/points_filtered"):
    n.create_subscription(PointCloud2, t, lambda m, t=t: got.__setitem__(t, m), 2)
for t in ("lidar/scan", "lidar/scan_filtered"):
    n.create_subscription(LaserScan, t, lambda m, t=t: got.__setitem__(t, m), 2)
t0 = time.time()
while len(got) < 6 and time.time() - t0 < 300:
    rclpy.spin_once(n, timeout_sec=0.1)
print("topics:", sorted(got))

def to_base(P, frame):
    tf = buf.lookup_transform("base_footprint", frame, Time(), timeout=rclpy.duration.Duration(seconds=5)).transform
    x, y, z, w = tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return P @ R.T + np.array([tf.translation.x, tf.translation.y, tf.translation.z])

def inside(B):
    return ((B[:, 0] > BOX["min_x"]) & (B[:, 0] < BOX["max_x"]) & (B[:, 1] > BOX["min_y"]) & (B[:, 1] < BOX["max_y"])
            & (B[:, 2] > BOX["min_z"]) & (B[:, 2] < BOX["max_z"]))

def cloud(m):
    P = np.array([tuple(p)[:3] for p in pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)], float).reshape(-1, 3)
    return to_base(P[np.isfinite(P).all(1)], m.header.frame_id)

def scan(m):
    r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
    P = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())], 1)
    return to_base(P, m.header.frame_id)

fails = 0
for raw, filt, conv in (("lidar/points", "lidar/points_filtered", cloud),
                        ("depth_camera/points", "depth_camera/points_filtered", cloud),
                        ("lidar/scan", "lidar/scan_filtered", scan)):
    A, B = conv(got[raw]), conv(got[filt])
    ia, ib = inside(A).sum(), inside(B).sum()
    oa, ob = (~inside(A)).sum(), (~inside(B)).sum()
    ok = ib == 0 and ob >= 0.9 * oa
    fails += not ok
    print(f"  [{'PASS' if ok else 'FAIL'}] {raw:22s} on rover: {ia:6d} -> {ib:4d}   environment kept: {oa:6d} -> {ob:6d}   "
          f"frame {got[filt].header.frame_id}")
sys.exit(1 if fails else 0)
