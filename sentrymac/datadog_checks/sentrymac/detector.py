"""
Simplified vibration/motion detector for the SentryMac Datadog check.

Stripped-down version of VibrationDetector from motion_live.py.  Keeps only
the algorithms needed for a Datadog check (no PyWavelets, no TUI state):

  - High-pass IIR filter (α=0.95) for gravity removal
  - STA/LTA (single timescale: STA=3 samples, LTA=100 samples)
  - Severity classifier: micro_vib → vibration → shock → major_shock
  - Debounce: configurable cooldown between emitted events (default 30 s)
  - Mahony AHRS quaternion filter for orientation (roll/pitch/yaw)
  - Sensitivity presets: low / medium / high
"""

import math
import time
from collections import deque

# ---------------------------------------------------------------------------
# Sensitivity presets
# ---------------------------------------------------------------------------
PRESETS = {
    'low': {
        'sta_lta_thresh': 5.0,
        'mag_floor': 0.01,
    },
    'medium': {
        'sta_lta_thresh': 3.0,
        'mag_floor': 0.003,
    },
    'high': {
        'sta_lta_thresh': 2.0,
        'mag_floor': 0.001,
    },
}

# Severity classification thresholds (amplitude in g)
_SEV_MAJOR_SHOCK = 0.05
_SEV_SHOCK = 0.02
_SEV_VIBRATION = 0.003


class VibrationDetector:
    """
    Lightweight motion/vibration detector suitable for a Datadog Agent check.

    Call :meth:`process` for each accelerometer sample and :meth:`process_gyro`
    for each gyroscope sample.  :meth:`process` returns an event dict when a
    motion event is classified (subject to debounce), or ``None``.
    """

    def __init__(self, fs=100, sensitivity='medium', debounce_seconds=30.0):
        self.fs = fs
        preset = PRESETS.get(sensitivity, PRESETS['medium'])
        self.sta_lta_thresh = preset['sta_lta_thresh']
        self.mag_floor = preset['mag_floor']
        self.debounce_seconds = debounce_seconds

        # High-pass IIR state
        self.hp_alpha = 0.95
        self._hp_prev_raw = [0.0, 0.0, 0.0]
        self._hp_prev_out = [0.0, 0.0, 0.0]
        self._hp_ready = False

        # STA / LTA (single timescale)
        self._sta_n = 3          # ~0.03 s at 100 Hz
        self._lta_n = 100        # ~1.0  s at 100 Hz
        self._sta = 0.0
        self._lta = 1e-10
        self.sta_lta_latest = 1.0
        self._sta_lta_active = False

        # Running RMS / peak over the check interval
        self._rms_acc = 0.0      # sum of squares
        self._rms_count = 0
        self._peak_val = 0.0
        self.rms = 0.0
        self.peak = 0.0

        # Debounce
        self._last_event_time = 0.0

        # Sample counter
        self.sample_count = 0

        # Gyroscope latest values (deg/s)
        self.gyro_latest = (0.0, 0.0, 0.0)

        # Mahony AHRS quaternion orientation
        self._q = [1.0, 0.0, 0.0, 0.0]
        self._mahony_kp = 1.0
        self._mahony_ki = 0.05
        self._mahony_err_int = [0.0, 0.0, 0.0]
        self._orient_init = False

    # ------------------------------------------------------------------
    # Gyroscope input
    # ------------------------------------------------------------------

    def process_gyro(self, gx, gy, gz):
        """Store the latest gyroscope reading (deg/s)."""
        self.gyro_latest = (gx, gy, gz)

    # ------------------------------------------------------------------
    # Orientation (Mahony AHRS)
    # ------------------------------------------------------------------

    def _update_orientation(self, ax, ay, az):
        """Mahony AHRS filter: fuses accel (gravity) + gyro via quaternion."""
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 0.3:
            return

        gx = math.radians(self.gyro_latest[0])
        gy = math.radians(self.gyro_latest[1])
        gz = math.radians(self.gyro_latest[2])
        dt = 1.0 / self.fs

        if not self._orient_init:
            ax_n, ay_n, az_n = ax / a_norm, ay / a_norm, az / a_norm
            pitch0 = math.atan2(-ax_n, -az_n)
            roll0 = math.atan2(ay_n, -az_n)
            cp = math.cos(pitch0 * 0.5)
            sp = math.sin(pitch0 * 0.5)
            cr = math.cos(roll0 * 0.5)
            sr = math.sin(roll0 * 0.5)
            self._q = [cr * cp, sr * cp, cr * sp, -sr * sp]
            self._orient_init = True
            return

        q = self._q
        inv_norm = 1.0 / a_norm
        ax_n, ay_n, az_n = ax * inv_norm, ay * inv_norm, az * inv_norm

        qw, qx, qy, qz = q
        vx = 2.0 * (qx * qz - qw * qy)
        vy = 2.0 * (qw * qx + qy * qz)
        vz = qw * qw - qx * qx - qy * qy + qz * qz

        ex = (ay_n * (-vz) - az_n * (-vy))
        ey = (az_n * (-vx) - ax_n * (-vz))
        ez = (ax_n * (-vy) - ay_n * (-vx))

        self._mahony_err_int[0] += self._mahony_ki * ex * dt
        self._mahony_err_int[1] += self._mahony_ki * ey * dt
        self._mahony_err_int[2] += self._mahony_ki * ez * dt

        gx += self._mahony_kp * ex + self._mahony_err_int[0]
        gy += self._mahony_kp * ey + self._mahony_err_int[1]
        gz += self._mahony_kp * ez + self._mahony_err_int[2]

        hdt = 0.5 * dt
        dw = (-qx * gx - qy * gy - qz * gz) * hdt
        dx = ( qw * gx + qy * gz - qz * gy) * hdt
        dy = ( qw * gy - qx * gz + qz * gx) * hdt
        dz = ( qw * gz + qx * gy - qy * gx) * hdt

        qw += dw; qx += dx; qy += dy; qz += dz

        n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if n > 0:
            inv_n = 1.0 / n
            qw *= inv_n; qx *= inv_n; qy *= inv_n; qz *= inv_n

        self._q = [qw, qx, qy, qz]

    def get_orientation_degrees(self):
        """Return (roll, pitch, yaw) in degrees from the current quaternion."""
        qw, qx, qy, qz = self._q
        sin_r = 2.0 * (qw * qx + qy * qz)
        cos_r = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.degrees(math.atan2(sin_r, cos_r))

        sin_p = 2.0 * (qw * qy - qz * qx)
        sin_p = max(-1.0, min(1.0, sin_p))
        pitch = math.degrees(math.asin(sin_p))

        sin_y = 2.0 * (qw * qz + qx * qy)
        cos_y = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.degrees(math.atan2(sin_y, cos_y))

        return roll, pitch, yaw

    # ------------------------------------------------------------------
    # Main per-sample processing
    # ------------------------------------------------------------------

    def process(self, ax, ay, az, t_now):
        """
        Process one accelerometer sample.

        Returns an event dict if a motion event was classified (and not
        debounced), otherwise ``None``.
        """
        self.sample_count += 1
        self._update_orientation(ax, ay, az)

        # -- High-pass filter (remove gravity) --
        if not self._hp_ready:
            self._hp_prev_raw = [ax, ay, az]
            self._hp_prev_out = [0.0, 0.0, 0.0]
            self._hp_ready = True
            return None

        a = self.hp_alpha
        hx = a * (self._hp_prev_out[0] + ax - self._hp_prev_raw[0])
        hy = a * (self._hp_prev_out[1] + ay - self._hp_prev_raw[1])
        hz = a * (self._hp_prev_out[2] + az - self._hp_prev_raw[2])
        self._hp_prev_raw = [ax, ay, az]
        self._hp_prev_out = [hx, hy, hz]

        mag = math.sqrt(hx * hx + hy * hy + hz * hz)

        # -- Update running RMS / peak --
        self._rms_acc += mag * mag
        self._rms_count += 1
        self.rms = math.sqrt(self._rms_acc / self._rms_count)
        if mag > self._peak_val:
            self._peak_val = mag
        self.peak = self._peak_val

        # -- STA / LTA --
        e = mag * mag
        self._sta += (e - self._sta) / self._sta_n
        self._lta += (e - self._lta) / self._lta_n
        ratio = self._sta / (self._lta + 1e-30)
        self.sta_lta_latest = ratio

        # -- Trigger logic --
        triggered = False
        if ratio > self.sta_lta_thresh and mag > self.mag_floor:
            if not self._sta_lta_active:
                self._sta_lta_active = True
                triggered = True
        elif ratio < self.sta_lta_thresh * 0.5:
            self._sta_lta_active = False

        if not triggered:
            return None

        # -- Debounce --
        if (t_now - self._last_event_time) < self.debounce_seconds:
            return None
        self._last_event_time = t_now

        # -- Classify severity --
        severity, label = self._classify(mag)

        return {
            'timestamp': t_now,
            'severity': severity,
            'severity_label': label,
            'amplitude': mag,
            'sta_lta_ratio': ratio,
        }

    # ------------------------------------------------------------------
    # Severity classifier
    # ------------------------------------------------------------------

    @staticmethod
    def _classify(amplitude):
        """Return (severity_key, human_label) for the given vibration amplitude."""
        if amplitude > _SEV_MAJOR_SHOCK:
            return 'major_shock', 'Major shock'
        elif amplitude > _SEV_SHOCK:
            return 'shock', 'Shock'
        elif amplitude > _SEV_VIBRATION:
            return 'vibration', 'Vibration'
        else:
            return 'micro_vib', 'Micro-vibration'

    # ------------------------------------------------------------------
    # Interval reset (call between check() intervals to reset RMS/peak)
    # ------------------------------------------------------------------

    def reset_interval(self):
        """Reset per-interval accumulators (RMS, peak). Call at start of each check()."""
        self._rms_acc = 0.0
        self._rms_count = 0
        self._peak_val = 0.0
        self.rms = 0.0
        self.peak = 0.0
