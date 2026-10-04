from __future__ import annotations

import math
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QLabel, QScrollArea, QToolTip, QWidget

from . import vector_math


class ArenaWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 500)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.snapshot = None
        self.config = None
        self.trail = deque(maxlen=900)
        self.show_vectors = True
        # Zoom 1 shows a 1.3 m half-span centred on the origin. Above that the
        # camera follows the robot so it stays in view.
        self.zoom = 3.0
        self.selected = None
        self.hovered = None
        self._hits = []  # (key, start, end) in widget coordinates, from the last paint
        self._legend_hits = []  # (key, QRectF) for the live legend rows
        self.banner = None
        self.steering = None
        # Scrollable so long derivations fit; it captures clicks and wheel
        # events over its own area only.
        self.overlay = QScrollArea(self)
        self.overlay.setObjectName("math_overlay")
        self.overlay.setWidgetResizable(True)
        self.overlay.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.overlay_text = QLabel()
        self.overlay_text.setObjectName("math_overlay_text")
        self.overlay_text.setTextFormat(Qt.TextFormat.RichText)
        self.overlay_text.setWordWrap(True)
        self.overlay_text.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.overlay.setWidget(self.overlay_text)
        self.overlay.hide()

    OVERLAY_WIDTH = 470

    def set_show_vectors(self, show):
        self.show_vectors = bool(show)
        if not self.show_vectors:
            self.select(None)
        self.update()

    def select(self, key):
        self.selected = key
        self._refresh_overlay()
        self.update()

    def _refresh_overlay(self):
        if self.selected is None or self.snapshot is None or self.config is None:
            self.overlay.hide()
            return
        self.overlay_text.setText(vector_math.to_html(vector_math.derive(self.selected, self.snapshot, self.config, self.steering)))
        self._place_overlay()
        self.overlay.show()

    def _place_overlay(self):
        width = min(self.OVERLAY_WIDTH, self.width() - 40)
        content = self.overlay_text.heightForWidth(width - 24) + 24
        self.overlay.setFixedSize(width, max(120, min(content, self.height() - 20)))
        self.overlay.move(self.width() - width - 10, 10)

    def resizeEvent(self, event):
        if self.overlay.isVisible():
            self._place_overlay()
        super().resizeEvent(event)

    def wheelEvent(self, event):
        steps = event.angleDelta().y() / 120.0
        self.zoom = min(40.0, max(0.5, self.zoom * 1.2 ** steps))
        self.update()

    def set_steering(self, state):
        self.steering = state
        if self.selected == "steering":
            self._refresh_overlay()

    def set_banner(self, text):
        """Short status shown over the arena, e.g. why the motors are off."""
        if text != self.banner:
            self.banner = text
            self.update()

    def _hit(self, pos):
        for key, rect in self._legend_hits:
            if rect.contains(pos):
                return key
        best, best_distance = None, 9.0
        for key, start, end in self._hits:
            distance = _segment_distance(pos, start, end)
            if distance < best_distance:
                best, best_distance = key, distance
        return best

    def mouseMoveEvent(self, event):
        key = self._hit(event.position()) if self.show_vectors else None
        if key != self.hovered:
            self.hovered = key
            self.setCursor(Qt.CursorShape.PointingHandCursor if key else Qt.CursorShape.ArrowCursor)
            self.update()
        if key and self.snapshot is not None and self.config is not None:
            QToolTip.showText(event.globalPosition().toPoint(),
                              vector_math.summary(key, self.snapshot, self.config, self.steering), self)
        else:
            QToolTip.hideText()

    def leaveEvent(self, event):
        self.hovered = None
        self.update()
        super().leaveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            key = self._hit(event.position()) if self.show_vectors else None
            self.select(None if key == self.selected else key)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.select(None)
        else:
            super().keyPressEvent(event)

    def set_config(self, config):
        self.config = config
        self.trail.clear()
        self._refresh_overlay()
        self.update()

    def set_snapshot(self, snapshot):
        self.snapshot = snapshot
        point = (snapshot.x_m, snapshot.y_m)
        if not self.trail or point != self.trail[-1]:
            self.trail.append(point)
        if self.selected is not None:
            self._refresh_overlay()
        self.update()

    def _steering_legend(self):
        st = self.steering
        if st is None or st.strength <= 0:
            return "—"
        push = "…" if st.push_rad is None else f"{math.degrees(st.push_rad):+.0f}°"
        return f"{math.degrees(st.desired_rad):+.0f}° → {push}"

    def _draw_steering_ghost(self, p, center, radius):
        """World-aligned demand as a ghost on the robot, plus the measured push."""
        st = self.steering
        length = radius + 60.0
        if st is None or st.strength <= 0:
            self._hits.append(("steering", center, center))
            return
        tip = center + QPointF(math.cos(st.desired_rad), -math.sin(st.desired_rad)) * (length * (0.4 + 0.6 * st.strength))
        ghost = QPen(QColor(255, 209, 102, 170), 4, Qt.PenStyle.DashLine); ghost.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(ghost); p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(center, radius + 6, radius + 6)
        self._vector(p, "steering", center, tip, 2)
        p.setPen(ghost); p.drawLine(center, tip)
        if st.push_rad is not None:
            push_tip = center + QPointF(math.cos(st.push_rad), -math.sin(st.push_rad)) * (length * 0.85)
            dotted = QPen(QColor("#ef5da8"), 2, Qt.PenStyle.DotLine)
            p.setPen(dotted); p.drawLine(center, push_tip)
            p.setBrush(QColor("#ef5da8")); p.setPen(Qt.PenStyle.NoPen); p.drawEllipse(push_tip, 4, 4)

    def _vector(self, p, key, start, end, width):
        """Draw a selectable arrow and record its hit segment in widget space."""
        transform = p.transform()
        self._hits.append((key, transform.map(start), transform.map(end)))
        if key in (self.selected, self.hovered):
            glow = QPen(QColor(vector_math.COLORS[key]), width + (8 if key == self.selected else 5))
            glow.setCapStyle(Qt.PenCapStyle.RoundCap)
            glow_color = glow.color(); glow_color.setAlpha(90 if key == self.selected else 55); glow.setColor(glow_color)
            p.setPen(glow); p.drawLine(start, end)
        _arrow(p, start, end, vector_math.COLORS[key], width)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0b1018"))
        self._hits = []
        self._legend_hits = []
        # Keep the robot clear of the math overlay when it is open.
        visible_width = self.width() - (self.overlay.width() + 20 if self.overlay.isVisible() else 0)
        cx, cy = max(visible_width, 200) / 2, self.height() / 2
        scale = min(self.width(), self.height()) / 2.6 * self.zoom
        follow = self.snapshot is not None and self.zoom > 1.0
        cam_x, cam_y = (self.snapshot.x_m, self.snapshot.y_m) if follow else (0.0, 0.0)
        def world(x_m, y_m):
            return QPointF(cx + (x_m - cam_x) * scale, cy - (y_m - cam_y) * scale)
        spacing_m = next((m for m in (0.01, 0.02, 0.05, 0.1, 0.25, 0.5, 1.0) if m * scale >= 30), 1.0)
        p.setPen(QPen(QColor("#182334"), 1))
        gx = math.floor((cam_x - cx / scale) / spacing_m) * spacing_m
        while gx <= cam_x + (self.width() - cx) / scale:
            x = world(gx, 0).x(); p.drawLine(int(x), 0, int(x), self.height()); gx += spacing_m
        gy = math.floor((cam_y - cy / scale) / spacing_m) * spacing_m
        while gy <= cam_y + cy / scale:
            y = world(0, gy).y(); p.drawLine(0, int(y), self.width(), int(y)); gy += spacing_m
        p.setPen(QPen(QColor("#2e4058"), 1))
        origin = world(0.0, 0.0)
        p.drawLine(0, int(origin.y()), self.width(), int(origin.y()))
        p.drawLine(int(origin.x()), 0, int(origin.x()), self.height())
        p.setPen(QColor("#7f91a8"))
        p.drawText(self.width() - 230, self.height() - 14,
                   f"grid {spacing_m * 100:g} cm · scroll to zoom ({self.zoom:.1f}×)")
        if self.snapshot is None or self.config is None:
            p.setPen(QColor("#8090a6")); p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "WAITING FOR SIMULATOR")
            return
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
        # Sensor position is fixed on body +X. angle_rad rotates only its axes.
        sensor = QPointF(sensor_r, 0)
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor("#ef5da8")); p.drawEllipse(sensor, 4, 4)
        axis = 11.0
        p.setPen(QPen(QColor("#ef5da8"), 1))
        p.drawLine(sensor, sensor + QPointF(axis * math.cos(angle), -axis * math.sin(angle)))
        p.setPen(QPen(QColor("#8f5def"), 1))
        p.drawLine(sensor, sensor + QPointF(axis * math.sin(angle), axis * math.cos(angle)))
        if self.show_vectors:
            # Wheel A sits at body +X and drives along body +Y; wheel B sits at
            # body -X and drives along body -Y. Painter Y points down, so body +Y
            # is local -Y. Wheel arrows show the motor command (full length = 1);
            # the centre arrow shows net tire force (full length = traction limit).
            # A wheel's hit segment always spans the wheel itself so it stays
            # clickable at zero command.
            full = max(70.0, radius * 1.4)
            # Faint full-scale rails keep each vector visible and hoverable at 0.
            rail = QPen(QColor("#33475f"), 1, Qt.PenStyle.DashLine)
            p.setPen(rail)
            p.drawLine(QPointF(wheel_offset, 0), QPointF(wheel_offset, -full))
            p.drawLine(QPointF(-wheel_offset, 0), QPointF(-wheel_offset, full))
            p.drawLine(QPointF(0, full * 0.5), QPointF(0, -full * 0.5))
            self._hits.append(("net_force", p.transform().map(QPointF(0, full * 0.5)),
                               p.transform().map(QPointF(0, -full * 0.5))))
            command_a = s.firmware.output.wheel_a * full
            command_b = s.firmware.output.wheel_b * full
            self._hits.append(("wheel_a", p.transform().map(QPointF(wheel_offset, -wheel_r)),
                               p.transform().map(QPointF(wheel_offset, wheel_r))))
            self._hits.append(("wheel_b", p.transform().map(QPointF(-wheel_offset, -wheel_r)),
                               p.transform().map(QPointF(-wheel_offset, wheel_r))))
            self._vector(p, "wheel_a", QPointF(wheel_offset, 0), QPointF(wheel_offset, -command_a), 3)
            self._vector(p, "wheel_b", QPointF(-wheel_offset, 0), QPointF(-wheel_offset, command_b), 3)
            limit = max(1e-6, cfg.physical.tire_friction_coefficient * max(
                s.wheel_a.normal_force_n, s.wheel_b.normal_force_n, 1e-6))
            net = (s.wheel_a.applied_force_n - s.wheel_b.applied_force_n) / limit * full
            self._vector(p, "net_force", QPointF(0, 0), QPointF(0, -net), 3)
        p.restore()
        if self.show_vectors:
            # World-frame chassis velocity; 1 m/s spans 200 px, with a 30 px
            # floor so slow drift stays visible and hoverable.
            speed = math.hypot(s.vx_mps, s.vy_mps)
            length = max(30.0, speed * 200.0) if speed > 1e-4 else 0.0
            velocity = QPointF(s.vx_mps, -s.vy_mps) * (length / speed if speed > 1e-4 else 0.0)
            self._vector(p, "velocity", center, center + velocity, 2)
            self._draw_steering_ghost(p, center, radius)
            # Live legend: each row shows the vector's current value and is
            # hoverable/clickable like the arrow itself.
            values = {
                "wheel_a": f"{s.firmware.output.wheel_a:.3f}",
                "wheel_b": f"{s.firmware.output.wheel_b:.3f}",
                "net_force": f"{s.wheel_a.applied_force_n - s.wheel_b.applied_force_n:+.3f} N",
                "velocity": f"{speed * 1000:.1f} mm/s",
                "steering": self._steering_legend(),
            }
            metrics = p.fontMetrics()
            y = self.height() - 14
            for key in reversed(vector_math.VECTORS):
                label = vector_math.TITLES[key]
                rect = QRectF(10, y - metrics.ascent() - 3, 250, metrics.height() + 4)
                if key in (self.selected, self.hovered):
                    p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor(28, 44, 64, 220)); p.drawRoundedRect(rect, 4, 4)
                p.setPen(QColor(vector_math.COLORS[key])); p.drawText(16, y, label)
                p.setPen(QColor("#d6e0ec")); p.drawText(16 + 160, y, values[key])
                self._legend_hits.append((key, rect))
                y -= 20
            p.setPen(QColor("#7f91a8")); p.drawText(16, y, "hover or click a vector or a row below")
        if self.banner:
            p.setFont(p.font()); metrics = p.fontMetrics()
            width = metrics.horizontalAdvance(self.banner) + 28
            rect = QRectF(16, 58, width, metrics.height() + 12)
            p.setPen(QPen(QColor("#ff718d"), 1)); p.setBrush(QColor(60, 20, 32, 230)); p.drawRoundedRect(rect, 5, 5)
            p.setPen(QColor("#ffc2cf")); p.drawText(rect, Qt.AlignmentFlag.AlignCenter, self.banner)
        p.setPen(QColor("#b9c7d8"))
        p.drawText(16, 25, f"x {s.x_m:+.3f} m   y {s.y_m:+.3f} m   heading {s.heading_rad:+.3f} rad")
        p.drawText(16, 45, f"actual spin {s.spin_rad_s:+.1f} rad/s")


def _segment_distance(point, start, end):
    dx, dy = end.x() - start.x(), end.y() - start.y()
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-9:
        return math.hypot(point.x() - start.x(), point.y() - start.y())
    t = max(0.0, min(1.0, ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / length_sq))
    return math.hypot(point.x() - (start.x() + t * dx), point.y() - (start.y() + t * dy))


def _arrow(p, start, end, color, width=2):
    dx, dy = end.x() - start.x(), end.y() - start.y()
    length = math.hypot(dx, dy)
    if length < 2.0:
        return
    pen = QPen(QColor(color), width); pen.setCapStyle(Qt.PenCapStyle.RoundCap); p.setPen(pen)
    p.drawLine(start, end)
    ux, uy = dx / length, dy / length
    head = min(9.0, length * 0.4)
    for side in (1, -1):
        p.drawLine(end, QPointF(end.x() - head * (ux * 0.87 - side * uy * 0.5),
                                end.y() - head * (uy * 0.87 + side * ux * 0.5)))


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
        area = QRectF(48, 30, self.width() - 62, self.height() - 60)
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
        x = 58
        for text, color in (("actual spin", "#58c7f3"), ("estimated spin", "#ef5da8"),
                            ("wheel A", "#f5a44c"), ("wheel B", "#8bd17c")):
            p.setPen(QColor(color)); p.drawText(x, 20, text)
            x += p.fontMetrics().horizontalAdvance(text) + 16


class DirectionDial(QWidget):
    """360° translation demand: angle from the +X axis, strength from the centre."""

    changed = Signal(float, float)  # angle in degrees (CCW from +X), strength 0..1

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self.angle_deg = 0.0
        self.strength = 0.0
        self.velocity = None
        self.setToolTip(
            "Drag to set translation direction (angle) and strength (distance from centre).\n"
            "Hold Shift to snap to 15°. Right-click or double-click to centre (stop translating).\n"
            "0° is +X and 90° is +Y in the firmware's phase frame; it matches the arena only\n"
            "when the translation phase offset is calibrated (see scenario.json).")

    def set_value(self, angle_deg, strength):
        self.angle_deg, self.strength = angle_deg, strength
        self.update()

    def set_velocity(self, vx, vy):
        self.velocity = (vx, vy) if math.hypot(vx, vy) > 1e-4 else None
        self.update()

    def _geometry(self):
        side = min(self.width(), self.height())
        return QPointF(self.width() / 2, self.height() / 2), side / 2 - 26

    def _set_from(self, pos, snap):
        center, radius = self._geometry()
        dx, dy = pos.x() - center.x(), center.y() - pos.y()
        angle = math.degrees(math.atan2(dy, dx))
        if snap:
            angle = round(angle / 15.0) * 15.0
            angle = -180.0 if angle > 180.0 else angle
        self.angle_deg = angle
        self.strength = min(1.0, math.hypot(dx, dy) / radius)
        self.update()
        self.changed.emit(self.angle_deg, self.strength)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.RightButton:
            self.strength = 0.0; self.update(); self.changed.emit(self.angle_deg, 0.0)
        elif event.button() == Qt.MouseButton.LeftButton:
            self._set_from(event.position(), event.modifiers() & Qt.KeyboardModifier.ShiftModifier)

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MouseButton.LeftButton:
            self._set_from(event.position(), event.modifiers() & Qt.KeyboardModifier.ShiftModifier)

    def mouseDoubleClickEvent(self, event):
        self.strength = 0.0; self.update(); self.changed.emit(self.angle_deg, 0.0)

    def paintEvent(self, _event):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        center, radius = self._geometry()
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor("#0e1520"))
        p.drawEllipse(center, radius + 22, radius + 22)
        p.setBrush(Qt.BrushStyle.NoBrush)
        for fraction in (0.25, 0.5, 0.75):
            p.setPen(QPen(QColor("#1d2b3d"), 1)); p.drawEllipse(center, radius * fraction, radius * fraction)
        p.setPen(QPen(QColor("#38516d"), 2)); p.drawEllipse(center, radius, radius)
        for degree in range(0, 360, 15):
            rad = math.radians(degree)
            major = degree % 45 == 0
            inner = radius - (10 if major else 5)
            p.setPen(QPen(QColor("#5b7591" if major else "#2e4058"), 2 if major else 1))
            p.drawLine(QPointF(center.x() + inner * math.cos(rad), center.y() - inner * math.sin(rad)),
                       QPointF(center.x() + radius * math.cos(rad), center.y() - radius * math.sin(rad)))
            if major:
                label = f"{degree if degree <= 180 else degree - 360}°"
                lx = center.x() + (radius + 13) * math.cos(rad)
                ly = center.y() - (radius + 13) * math.sin(rad)
                p.setPen(QColor("#8a9bb2"))
                p.drawText(QRectF(lx - 20, ly - 8, 40, 16), Qt.AlignmentFlag.AlignCenter, label)
        p.setPen(QPen(QColor("#1d2b3d"), 1))
        p.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()))
        p.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius))
        if self.velocity is not None:
            rad = math.atan2(self.velocity[1], self.velocity[0])
            tip = QPointF(center.x() + radius * math.cos(rad), center.y() - radius * math.sin(rad))
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor("#76d5f7"))
            p.drawEllipse(tip, 5, 5)
        rad = math.radians(self.angle_deg)
        knob = QPointF(center.x() + self.strength * radius * math.cos(rad),
                       center.y() - self.strength * radius * math.sin(rad))
        _arrow(p, center, knob, "#ffd166", 3)
        p.setPen(QPen(QColor("#ffd166"), 2)); p.setBrush(QColor("#3a3320"))
        p.drawEllipse(knob, 8, 8)
        p.setPen(QColor("#d6e0ec"))
        p.drawText(QRectF(center.x() - 60, center.y() + radius * 0.35, 120, 18), Qt.AlignmentFlag.AlignCenter,
                   f"{self.angle_deg:+.0f}° · {self.strength:.0%}")
