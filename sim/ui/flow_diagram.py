"""Live block diagram of the control loop: operator → receiver → firmware →
motors → tires → chassis → accelerometer → firmware, with real values."""
from __future__ import annotations

import math
from dataclasses import dataclass

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QToolTip, QWidget

from . import vector_math

DOMAIN_COLORS = {
    "operator": "#ffd166",
    "transport": "#8a9bb2",
    "sensor": "#ef5da8",
    "firmware": "#76d5f7",
    "physics": "#f5a44c",
}


@dataclass(frozen=True)
class Block:
    key: str
    title: str
    domain: str
    col: int
    row: int
    hint: str
    opens: str | None = None  # arena vector whose derivation explains this block


WHY = {
    "operator": 'You choose spin speed (for hitting power) and a direction to drive.',
    "steering": 'A real driver corrects by eye; this stands in so the dial means real-world directions.',
    "receiver": 'Real radios add delay, so the firmware always acts on a slightly old command.',
    "accelerometer": "It is the robot's only sense of its own rotation.",
    "spin_pi": 'Steady spin speed is what makes hits, and the steering timing, predictable.',
    "estimator": "Turns the sensor's outward push into a speed the controller can use.",
    "translation": 'A spinning robot can only drive by pulsing its wheels in time with its rotation.',
    "phase": 'Knowing which way it faces is what lets the firmware time those pulses.',
    "mixer": 'Splits one spin throttle and one push into two wheel commands.',
    "motors": "Motors can't change speed instantly; this lag is why steering needs a timing correction.",
    "tires": 'Grip is what turns wheel speed into force on the floor.',
    "chassis": 'Where every force ends up: the real motion, which the accelerometer then measures.',
}


BLOCKS = (
    Block("operator", "Operator", "operator", 0, 0,
          "Your live controls: spin slider and direction dial (world frame).", "steering"),
    Block("steering", "World steering", "operator", 1, 0,
          "Sim-only driver assist: rotates the dial into the firmware's phase frame using the true\n"
          "heading (drift) and a spin-dependent lag model, plus a slow trim from the measured push.", "steering"),
    Block("receiver", "RC receiver", "transport", 0, 1,
          "Radio link: the firmware sees each command after the receiver latency\n"
          "(⚙ Simulation › Receiver latency).", "wheel_a"),
    Block("accelerometer", "Accelerometer", "sensor", 2, 1,
          "Measures the real acceleration at the sensor, with noise, bias, latency and a range limit\n"
          "(⚙ Physical sensor). Saturating it disarms the firmware.", "wheel_a"),
    Block("spin_pi", "Spin controller", "firmware", 0, 2,
          "PI loop: compares target spin with the estimate and sets the common throttle c\n"
          "that both wheels share.", "wheel_a"),
    Block("estimator", "Spin estimator", "firmware", 1, 2,
          "Turns radial acceleration into spin: |ω̂| = √(ā_r / r_s), after a low-pass filter.", "wheel_a"),
    Block("translation", "Translation", "firmware", 0, 3,
          "Times a push to the rotation: m = gain·|T|·cos(φ_A − ψ), capped so both wheels stay in 0…1.", "wheel_a"),
    Block("phase", "Phase integrator", "firmware", 1, 3,
          "Integrates the spin estimate into a rotation angle: φ̂ += ω̂·Δt.\n"
          "Any spin-estimate error accumulates here as heading drift.", "wheel_a"),
    Block("mixer", "Mixer", "firmware", 0, 4,
          "Wheel throttles: u_A = c + m, u_B = c − m, each kept within 0…1.", "wheel_a"),
    Block("motors", "Motors", "physics", 0, 5,
          "ESC + motor model: throttle → current → wheel speed, after the actuator latency\n"
          "and the motor time constant.", "wheel_a"),
    Block("tires", "Tires", "physics", 1, 5,
          "Contact model: slip between wheel surface and ground → force, capped by μ·N.", "net_force"),
    Block("chassis", "Chassis", "physics", 2, 5,
          "Rigid body: forces and torque → spin, heading, velocity and position.", "velocity"),
)
BLOCK_BY_KEY = {block.key: block for block in BLOCKS}
FIRMWARE_KEYS = ("spin_pi", "estimator", "translation", "phase", "mixer")

# (source, target, source side, target side, style, label key)
EDGES = (
    ("operator", "steering", "right", "left", "solid", "dial"),
    ("steering", "receiver", "bottom", "top", "solid", "sent"),
    ("receiver", "spin_pi", "bottom", "top", "solid", "spin"),
    ("receiver", "translation", "left", "left", "solid", "translate"),
    ("accelerometer", "estimator", "bottom", "right", "solid", "accel"),
    ("estimator", "spin_pi", "left", "right", "solid", "omega"),
    ("estimator", "phase", "bottom", "top", "solid", "omega_short"),
    ("phase", "translation", "left", "right", "solid", "phase"),
    ("spin_pi", "translation", "bottom", "top", "solid", "common"),
    ("translation", "mixer", "bottom", "top", "solid", "c_m"),
    ("mixer", "motors", "bottom", "top", "solid", "throttle"),
    ("motors", "tires", "right", "left", "solid", "wheel_speed"),
    ("tires", "chassis", "right", "left", "solid", "force"),
    ("chassis", "accelerometer", "top", "bottom", "solid", "motion"),
    ("chassis", "steering", "right", "right", "dashed", "truth"),
)


def _deg(rad):
    return f"{math.degrees(rad):+.0f}°"


class FlowDiagram(QWidget):
    block_clicked = Signal(str)  # arena vector key

    ROWS = 6
    COLS = 3

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(520)
        self.setMouseTracking(True)
        self.snapshot = self.config = self.steering = None
        self.requested_spin = 0.0
        self._rects = {}
        self._hovered = None

    def set_state(self, snapshot, config, steering, requested_spin):
        self.snapshot, self.config, self.steering = snapshot, config, steering
        self.requested_spin = requested_spin
        if self.isVisible():
            self.update()

    # Layout -----------------------------------------------------------------

    def _layout(self):
        margin_x, margin_top, margin_bottom = 24.0, 6.0, 6.0
        width = self.width() - 2 * margin_x
        col_w = width / self.COLS
        row_h = (self.height() - margin_top - margin_bottom) / self.ROWS
        # Leave ~56 px between columns so horizontal signal labels fit.
        block_w, block_h = col_w - 56, min(58.0, row_h - 26)
        rects = {}
        for block in BLOCKS:
            x = margin_x + block.col * col_w + (col_w - block_w) / 2
            y = margin_top + block.row * row_h + (row_h - block_h) / 2
            rects[block.key] = QRectF(x, y, block_w, block_h)
        return rects

    @staticmethod
    def _port(rect, side):
        return {"top": QPointF(rect.center().x(), rect.top()),
                "bottom": QPointF(rect.center().x(), rect.bottom()),
                "left": QPointF(rect.left(), rect.center().y()),
                "right": QPointF(rect.right(), rect.center().y())}[side]

    def _route(self, src, dst, src_side, dst_side):
        a, b = self._port(src, src_side), self._port(dst, dst_side)
        if src_side == dst_side == "left":
            x = 10.0
            return [a, QPointF(x, a.y()), QPointF(x, b.y()), b]
        if src_side == dst_side == "right":
            x = self.width() - 10.0
            return [a, QPointF(x, a.y()), QPointF(x, b.y()), b]
        if {src_side, dst_side} == {"top", "bottom"}:
            if abs(a.x() - b.x()) < 1:
                return [a, b]
            mid = (a.y() + b.y()) / 2
            return [a, QPointF(a.x(), mid), QPointF(b.x(), mid), b]
        if {src_side, dst_side} == {"left", "right"}:
            if abs(a.y() - b.y()) < 1:
                return [a, b]
            mid = (a.x() + b.x()) / 2
            return [a, QPointF(mid, a.y()), QPointF(mid, b.y()), b]
        # Perpendicular sides: one elbow.
        if src_side in ("top", "bottom"):
            return [a, QPointF(a.x(), b.y()), b]
        return [a, QPointF(b.x(), a.y()), b]

    # Values -----------------------------------------------------------------

    def _lines(self, key):
        s, cfg, st = self.snapshot, self.config, self.steering
        c = cfg.firmware.controller
        t = s.firmware.controller
        cmd = s.consumed_command
        if key == "operator":
            strength = st.strength if st else 0.0
            dial = f"dial {_deg(st.desired_rad)} · {strength:.0%}" if st and strength > 0 else "dial centred"
            return [f"spin {self.requested_spin:.0%}", dial]
        if key == "steering":
            if st is None or not st.enabled:
                return ["off", "sends dial angle as-is"]
            return [f"drift {_deg(st.drift_rad)} lag {_deg(st.lag_rad)}", f"sends {_deg(st.sent_rad)}"]
        if key == "receiver":
            magnitude = min(1.0, math.hypot(cmd.translate_x, cmd.translate_y))
            return [f"+{cfg.command_latency_us / 1e3:g} ms · spin {cmd.spin:.0%}",
                    f"T {magnitude:.0%} @ {_deg(math.atan2(cmd.translate_y, cmd.translate_x))}"]
        if key == "accelerometer":
            a = s.consumed_acceleration
            limit = f"range ±{cfg.sensor.max_acceleration_mps2:.0f}"
            return [f"({a.x_mps2:+.0f}, {a.y_mps2:+.0f}) m/s²",
                    ("SATURATED · " if s.sensed_acceleration.saturated else "") + limit]
        if key == "estimator":
            return [f"ā_r {t.radial_accel_mps2:.0f} m/s²", f"ω̂ {t.spin_rad_s:+.1f} rad/s"]
        if key == "phase":
            return [f"φ̂ {_deg(t.phase_rad)}", "valid" if t.phase_valid else "invalid (too slow)"]
        if key == "spin_pi":
            target = cmd.spin * c.maximum_spin_rad_s
            return [f"ω* {target:.0f} e {t.spin_error_rad_s:+.1f}", f"c = {t.common_command:.3f}"]
        if key == "translation":
            angle = t.phase_rad + c.spin_direction * 0.5 * math.pi + c.translation_phase_offset_rad
            return [f"φ_A {_deg(math.remainder(angle, 2 * math.pi))}", f"m = {t.modulation_command:+.3f}"]
        if key == "mixer":
            return [f"u_A = {s.firmware.output.wheel_a:.3f}", f"u_B = {s.firmware.output.wheel_b:.3f}"]
        if key == "motors":
            return [f"ω_w {s.wheel_a.wheel_speed_rad_s:.0f}/{s.wheel_b.wheel_speed_rad_s:.0f} rad/s",
                    f"I {s.wheel_a.motor_current_a:.1f}/{s.wheel_b.motor_current_a:.1f} A"]
        if key == "tires":
            limited = s.wheel_a.traction_limited or s.wheel_b.traction_limited
            return [f"F {s.wheel_a.applied_force_n:+.2f}/{s.wheel_b.applied_force_n:+.2f} N",
                    "TRACTION LIMITED" if limited else f"slip {s.wheel_a.slip_mps:+.2f}/{s.wheel_b.slip_mps:+.2f}"]
        if key == "chassis":
            speed = math.hypot(s.vx_mps, s.vy_mps)
            return [f"ω {s.spin_rad_s:+.1f} θ {_deg(s.heading_rad)}",
                    f"{speed * 1000:.0f} mm/s @ {_deg(math.atan2(s.vy_mps, s.vx_mps))}"]
        return []

    def _edge_label(self, key):
        s, cfg, st = self.snapshot, self.config, self.steering
        t = s.firmware.controller
        cmd = s.consumed_command
        if key == "dial":
            return _deg(st.desired_rad) if st and st.strength > 0 else "—"
        if key == "sent":
            return f"ψ_fw {_deg(st.sent_rad)}" if st and st.strength > 0 else "ψ_fw —"
        if key == "spin":
            return f"spin {cmd.spin:.0%}"
        if key == "translate":
            return f"T ({cmd.translate_x:+.2f}, {cmd.translate_y:+.2f})"  # runs up the left bus
        if key == "accel":
            return f"a {math.hypot(s.consumed_acceleration.x_mps2, s.consumed_acceleration.y_mps2):.0f}"
        if key in ("omega", "omega_short"):
            return f"{abs(t.spin_rad_s):.0f}" if key == "omega" else f"ω̂ {abs(t.spin_rad_s):.0f}"
        if key == "phase":
            return _deg(t.phase_rad)
        if key == "common":
            return f"c {t.common_command:.2f}"
        if key == "c_m":
            return f"c {t.common_command:.2f}, m {t.modulation_command:+.2f}"
        if key == "throttle":
            return f"+{cfg.actuator_latency_us / 1e3:g} ms"
        if key == "wheel_speed":
            return "ω_w"
        if key == "force":
            return f"{s.wheel_a.applied_force_n - s.wheel_b.applied_force_n:+.2f} N"
        if key == "motion":
            return "true motion"
        if key == "truth":
            return "true θ, ω (sim only)"
        return ""

    # Painting ---------------------------------------------------------------

    def paintEvent(self, _event):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0e1520"))
        self._rects = rects = self._layout()
        if self.snapshot is None or self.config is None:
            p.setPen(QColor("#8090a6")); p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for simulator")
            return
        base = QFont(self.font()); small = QFont(base); small.setPointSizeF(max(7.0, base.pointSizeF() - 1.5))
        bold = QFont(base); bold.setBold(True)

        # Firmware container.
        box = rects[FIRMWARE_KEYS[0]]
        for key in FIRMWARE_KEYS[1:]:
            box = box.united(rects[key])
        box = box.adjusted(-10, -18, 10, 8)
        p.setPen(QPen(QColor("#2d4a66"), 1, Qt.PenStyle.DashLine)); p.setBrush(QColor(24, 40, 58, 120))
        p.drawRoundedRect(box, 8, 8)
        armed = self.snapshot.firmware.armed
        p.setFont(small); p.setPen(QColor("#76d5f7" if armed else "#ff718d"))
        caption = "FIRMWARE · " + ("ARMED" if armed else "DISARMED")
        p.drawText(QPointF(box.right() - 8 - p.fontMetrics().horizontalAdvance(caption), box.top() + 12), caption)

        # Edges under blocks.
        p.setFont(small)
        for source, target, src_side, dst_side, style, label_key in EDGES:
            points = self._route(rects[source], rects[target], src_side, dst_side)
            color = QColor("#ffd166" if style == "dashed" else "#4f6886")
            pen = QPen(color, 1.5, Qt.PenStyle.DashLine if style == "dashed" else Qt.PenStyle.SolidLine)
            p.setPen(pen); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPolyline(QPolygonF(points))
            self._arrowhead(p, points[-2], points[-1], color)
            self._label(p, points, self._edge_label(label_key), color)

        # Blocks.
        for block in BLOCKS:
            rect = rects[block.key]
            color = QColor(DOMAIN_COLORS[block.domain])
            fill = QColor(20, 31, 46) if block.key != self._hovered else QColor(30, 46, 66)
            p.setPen(QPen(color, 2 if block.key == self._hovered else 1.2)); p.setBrush(fill)
            p.drawRoundedRect(rect, 6, 6)
            p.setFont(bold); p.setPen(color)
            p.drawText(rect.adjusted(8, 4, -6, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop, block.title)
            p.setFont(small)
            y = rect.top() + 22
            for line in self._lines(block.key):
                warn = "SATURATED" in line or "LIMITED" in line or "invalid" in line
                p.setPen(QColor("#ff718d" if warn else "#d6e0ec"))
                p.drawText(QRectF(rect.left() + 8, y, rect.width() - 12, 16),
                           Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, line)
                y += 16
        p.setFont(base)

    @staticmethod
    def _arrowhead(p, before, tip, color):
        dx, dy = tip.x() - before.x(), tip.y() - before.y()
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length
        size = 7.0
        left = QPointF(tip.x() - size * (ux * 0.9 - uy * 0.5), tip.y() - size * (uy * 0.9 + ux * 0.5))
        right = QPointF(tip.x() - size * (ux * 0.9 + uy * 0.5), tip.y() - size * (uy * 0.9 - ux * 0.5))
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(color)
        p.drawPolygon(QPolygonF([tip, left, right]))

    @staticmethod
    def _label(p, points, text, color):
        if not text:
            return
        # Place on the longest segment; vertical runs get text written along them.
        best = max(range(len(points) - 1),
                   key=lambda i: math.hypot(points[i + 1].x() - points[i].x(), points[i + 1].y() - points[i].y()))
        a, b = points[best], points[best + 1]
        mid = QPointF((a.x() + b.x()) / 2, (a.y() + b.y()) / 2)
        vertical = abs(a.x() - b.x()) < 1 and abs(a.y() - b.y()) > p.fontMetrics().horizontalAdvance(text) + 16
        width = p.fontMetrics().horizontalAdvance(text) + 8
        p.save(); p.translate(mid)
        if vertical:
            p.rotate(-90)
        rect = QRectF(-width / 2, -8, width, 16)
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor(14, 21, 32, 230)); p.drawRoundedRect(rect, 3, 3)
        p.setPen(QColor("#ffd166") if color == QColor("#ffd166") else QColor("#9fb3c8"))
        p.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)
        p.restore()

    # Interaction ------------------------------------------------------------

    def _block_at(self, pos):
        for key, rect in self._rects.items():
            if rect.contains(pos):
                return key
        return None

    def mouseMoveEvent(self, event):
        key = self._block_at(event.position())
        if key != self._hovered:
            self._hovered = key
            self.setCursor(Qt.CursorShape.PointingHandCursor if key else Qt.CursorShape.ArrowCursor)
            self.update()
        if key:
            block = BLOCK_BY_KEY[key]
            more = (f"\nClick to open the {vector_math.TITLES[block.opens].lower()} math in the arena."
                    if block.opens else "")
            why = f"\nWhy: {WHY[key]}" if key in WHY else ""
            QToolTip.showText(event.globalPosition().toPoint(), block.hint + why + more, self)
        else:
            QToolTip.hideText()

    def leaveEvent(self, event):
        self._hovered = None
        self.update()
        super().leaveEvent(event)

    def mousePressEvent(self, event):
        key = self._block_at(event.position())
        if key and BLOCK_BY_KEY[key].opens:
            self.block_clicked.emit(BLOCK_BY_KEY[key].opens)
