from __future__ import annotations

import math
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget


class ArenaWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 500)
        self.snapshot = None
        self.config = None
        self.trail = deque(maxlen=900)

    def set_config(self, config):
        self.config = config
        self.trail.clear()
        self.update()

    def set_snapshot(self, snapshot):
        self.snapshot = snapshot
        point = (snapshot.x_m, snapshot.y_m)
        if not self.trail or point != self.trail[-1]:
            self.trail.append(point)
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0b1018"))
        cx, cy = self.width() / 2, self.height() / 2
        scale = min(self.width(), self.height()) / 2.6  # 1.3 m visible half-span
        p.setPen(QPen(QColor("#182334"), 1))
        spacing = max(30, scale * 0.25)
        x = cx % spacing
        while x < self.width():
            p.drawLine(int(x), 0, int(x), self.height()); x += spacing
        y = cy % spacing
        while y < self.height():
            p.drawLine(0, int(y), self.width(), int(y)); y += spacing
        p.setPen(QPen(QColor("#2e4058"), 1))
        p.drawLine(0, int(cy), self.width(), int(cy))
        p.drawLine(int(cx), 0, int(cx), self.height())
        if self.snapshot is None or self.config is None:
            p.setPen(QColor("#8090a6")); p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "WAITING FOR SIMULATOR")
            return
        def world(x_m, y_m):
            return QPointF(cx + x_m * scale, cy - y_m * scale)
        if len(self.trail) > 1:
            path = QPainterPath(world(*self.trail[0]))
            for point in list(self.trail)[1:]: path.lineTo(world(*point))
            p.setPen(QPen(QColor("#267fb7"), 2)); p.drawPath(path)
        s, cfg = self.snapshot, self.config
        center = world(s.x_m, s.y_m)
        radius = max(14.0, cfg.physical.body_radius_m * scale)
        p.save(); p.translate(center); p.rotate(-math.degrees(s.heading_rad))
        p.setPen(QPen(QColor("#5ec8f2"), 2)); p.setBrush(QColor("#17354a"))
        p.drawEllipse(QPointF(0, 0), radius, radius)
        p.setPen(QPen(QColor("#a5eeff"), 3)); p.drawLine(0, 0, radius, 0)
        wheel_offset = cfg.physical.wheel_offset_m * scale
        wheel_r = max(4.0, cfg.physical.wheel_radius_m * scale)
        p.setPen(QPen(QColor("#f5a44c"), 5))
        p.drawLine(QPointF(-wheel_offset, -wheel_r), QPointF(-wheel_offset, wheel_r))
        p.drawLine(QPointF(wheel_offset, -wheel_r), QPointF(wheel_offset, wheel_r))
        sensor_r = cfg.sensor.radius_m * scale
        angle = cfg.sensor.angle_rad
        sensor = QPointF(sensor_r * math.cos(angle), -sensor_r * math.sin(angle))
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor("#ef5da8")); p.drawEllipse(sensor, 4, 4)
        p.restore()
        p.setPen(QColor("#b9c7d8"))
        p.drawText(16, 25, f"x {s.x_m:+.3f} m   y {s.y_m:+.3f} m   heading {s.heading_rad:+.3f} rad")
        p.drawText(16, 45, f"actual spin {s.spin_rad_s:+.1f} rad/s")


class TelemetryPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(260)
        self.samples = deque(maxlen=600)

    def add_snapshot(self, s):
        self.samples.append((s.time_us / 1e6, s.spin_rad_s,
                             s.firmware.controller.spin_rad_s,
                             s.firmware.output.wheel_a, s.firmware.output.wheel_b))
        self.update()

    def clear(self):
        self.samples.clear(); self.update()

    def paintEvent(self, _event):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0e1520"))
        area = QRectF(48, 14, self.width() - 62, self.height() - 44)
        p.setPen(QPen(QColor("#26364a"), 1)); p.drawRect(area)
        if len(self.samples) < 2:
            p.setPen(QColor("#8090a6")); p.drawText(area, Qt.AlignmentFlag.AlignCenter, "Telemetry history")
            return
        samples = list(self.samples); t0, t1 = samples[0][0], samples[-1][0]
        span = max(0.001, t1 - t0)
        max_spin = max(10.0, max(abs(v) for row in samples for v in row[1:3]) * 1.1)
        def line(index, color, scale):
            path = QPainterPath()
            for i, row in enumerate(samples):
                x = area.left() + (row[0] - t0) / span * area.width()
                y = area.center().y() - row[index] / scale * area.height() * 0.46
                (path.moveTo if i == 0 else path.lineTo)(x, y)
            p.setPen(QPen(QColor(color), 2)); p.drawPath(path)
        line(1, "#58c7f3", max_spin); line(2, "#ef5da8", max_spin)
        line(3, "#f5a44c", 1.0); line(4, "#8bd17c", 1.0)
        p.setPen(QColor("#91a2b8")); p.drawText(48, self.height() - 10, f"{t0:.1f} s")
        p.drawText(self.width() - 78, self.height() - 10, f"{t1:.1f} s")
        p.setPen(QColor("#58c7f3")); p.drawText(58, 30, "actual spin")
        p.setPen(QColor("#ef5da8")); p.drawText(145, 30, "estimated spin")
        p.setPen(QColor("#f5a44c")); p.drawText(252, 30, "wheel A")
        p.setPen(QColor("#8bd17c")); p.drawText(315, 30, "wheel B")
