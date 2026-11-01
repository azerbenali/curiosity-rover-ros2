#!/usr/bin/env python3
"""Show one camera topic in a window, scaled to fill it. RGB images are shown as they are; depth
(32FC1, metres) is colourised near = red, far = blue, with a scale bar.

usage: cam_view.py <image topic> <window title>
"""
import re
import sys

import numpy as np
import rclpy
from PyQt5.QtCore import QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QLinearGradient, QPainter
from PyQt5.QtWidgets import QApplication, QWidget
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

MAX_DEPTH = 15.0                                            # m, far end of the colour scale


def turbo(t):
    """Google's Turbo colour map (polynomial fit), t in [0, 1] -> RGB in [0, 1]."""
    t = np.clip(t, 0, 1)
    r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))))
    g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))))
    b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))))
    return np.clip(np.stack([r, g, b], -1), 0, 1)


class View(QWidget):
    def __init__(self, topic, title):
        super().__init__()
        self.setWindowTitle(title)
        self.title, self.img, self.stamp, self.depth = title, None, 0.0, False
        self.node = rclpy.create_node("cam_view_" + re.sub(r"\W+", "_", topic).strip("_"))
        self.node.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.timer.start(10)
        self.resize(960, 720)

    def on_image(self, m):
        if m.encoding == "32FC1":
            d = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.step // 4)[:, : m.width]
            ok = np.isfinite(d)
            rgb = turbo(1 - np.where(ok, d, MAX_DEPTH) / MAX_DEPTH)
            rgb[~ok] = 0.12                                   # nothing within range (sky)
            rgb = (rgb * 255).astype(np.uint8)
            self.depth = True
        else:                                                 # rgb8
            rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3]
            rgb = rgb.reshape(m.height, m.width, 3)
        rgb = np.ascontiguousarray(rgb)
        self.img = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1] * 3, QImage.Format_RGB888).copy()
        self.stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.update()

    def tag(self, p, text, x, y, right=False):
        """Text on a dark rounded box at (x, y); right=True puts the box's right edge at x."""
        fm = p.fontMetrics()
        w, h = fm.horizontalAdvance(text) + 2 * self.pad, fm.height() + self.pad
        r = QRectF(x - w if right else x, y, w, h)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 140))
        p.drawRoundedRect(r, 6, 6)
        p.setPen(Qt.white)
        p.drawText(r, Qt.AlignCenter, text)

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), Qt.black)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        p.setRenderHint(QPainter.Antialiasing)
        if self.img is not None:
            s = self.img.size().scaled(self.size(), Qt.KeepAspectRatio)
            p.drawImage(QRectF((self.width() - s.width()) / 2, (self.height() - s.height()) / 2,
                               s.width(), s.height()), self.img)
        p.setFont(QFont("DejaVu Sans", max(10, self.height() // 42), QFont.Bold))
        m = self.pad = max(6, self.height() // 60)
        self.tag(p, self.title, m, m)
        self.tag(p, f"sim {self.stamp:5.1f} s", self.width() - m, m, right=True)
        if self.depth:                                        # scale bar: 0 m (red) ... MAX_DEPTH (blue)
            bw, bh, fh = self.width() * 0.4, self.height() / 40, p.fontMetrics().height()
            x, y = 2 * m, self.height() - 2 * m - bh
            g = QLinearGradient(x, 0, x + bw, 0)
            for t in np.linspace(0, 1, 11):
                g.setColorAt(t, QColor(*(turbo(1 - t) * 255).astype(int)))
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 140))
            p.drawRoundedRect(QRectF(x - m, y - fh - m, bw + 2 * m, bh + fh + 2 * m), 6, 6)
            p.setBrush(g)
            p.drawRect(QRectF(x, y, bw, bh))
            p.setPen(Qt.white)
            label = QRectF(x, y - fh - m / 2, bw, fh)
            p.drawText(label, Qt.AlignLeft, "0 m")
            p.drawText(label, Qt.AlignRight, f"{MAX_DEPTH:.0f} m")


def main():
    rclpy.init()
    app = QApplication(sys.argv[:1])
    v = View(sys.argv[1], sys.argv[2])
    v.show()
    app.exec_()
    v.node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
