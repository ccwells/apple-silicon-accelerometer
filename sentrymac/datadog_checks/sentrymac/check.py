"""
SentryMac — Datadog Agent custom check for MacBook physical security.

Reads acceleration, vibration, orientation, lid angle, and ambient light
from the Apple Silicon SPU IMU via a background sensor process, and emits
Datadog metrics, events, and service checks.
"""

import math
import multiprocessing
import multiprocessing.shared_memory
import struct
import time

try:
    from datadog_checks.base import AgentCheck
except ImportError:
    # Allow importing outside the Agent for testing
    class AgentCheck:
        OK, WARNING, CRITICAL, UNKNOWN = 0, 1, 2, 3
        def __init__(self, *args, **kwargs):
            self.instances = kwargs.get('instances', [{}])
        def gauge(self, name, value, tags=None): pass
        def service_check(self, name, status, tags=None, message=''): pass
        def event(self, event): pass
        @property
        def log(self):
            import logging
            return logging.getLogger('sentrymac')
        def warning(self, msg, *args): pass

from .spu_sensor import (
    is_apple_silicon,
    sensor_worker,
    shm_read_new,
    shm_read_new_gyro,
    shm_snap_read,
    SHM_NAME, SHM_NAME_GYRO, SHM_SIZE,
    SHM_NAME_ALS, SHM_ALS_SIZE,
    SHM_NAME_LID, SHM_LID_SIZE,
    SHM_SNAP_HDR, ALS_REPORT_LEN, ACCEL_SCALE,
)
from .detector import VibrationDetector

# Service check name
SERVICE_CHECK = 'sentrymac.sensor.status'

# Severity levels in ascending order (for min_severity filtering)
SEVERITY_ORDER = ['micro_vib', 'vibration', 'shock', 'major_shock']

# ALS lux field offset in the 122-byte report
_ALS_LUX_OFF = 40


class SentryMacCheck(AgentCheck):
    """
    Datadog Agent check that monitors the physical state of Apple Silicon
    MacBooks via the undocumented SPU IMU (Bosch BMI286).
    """

    def __init__(self, name, init_config, instances):
        super().__init__(name, init_config, instances)

        # Sensor worker process + shared memory handles
        self._worker = None
        self._restart_count = 0

        # Shared-memory objects (created on first check)
        self._shm_accel = None
        self._shm_gyro = None
        self._shm_als = None
        self._shm_lid = None

        # Cursors for ring buffer reads
        self._last_accel_total = 0
        self._last_gyro_total = 0
        self._last_als_count = 0
        self._last_lid_count = 0

        # Detector instance (created on first check)
        self._detector = None

        # Track first sample time for sample-rate calculation
        self._first_sample_time = None
        self._total_samples_seen = 0

        # Latest orientation state (for Mahony AHRS)
        self._latest_lid_angle = None
        self._latest_als_lux = None

    # ------------------------------------------------------------------
    # Config helpers
    # ------------------------------------------------------------------

    def _get_config(self, instance):
        """Parse and return configuration with defaults."""
        return {
            'sensitivity': instance.get('sensitivity', 'medium'),
            'motion_events_enabled': instance.get('motion_events_enabled', True),
            'motion_event_min_severity': instance.get('motion_event_min_severity', 'vibration'),
            'orientation_enabled': instance.get('orientation_enabled', True),
            'ambient_light_enabled': instance.get('ambient_light_enabled', True),
            'lid_angle_enabled': instance.get('lid_angle_enabled', True),
            'tags': instance.get('tags', []),
        }

    # ------------------------------------------------------------------
    # Sensor worker lifecycle
    # ------------------------------------------------------------------

    def _create_shm(self, name, size):
        """Create a zeroed shared-memory segment, unlinking any stale one first."""
        try:
            old = multiprocessing.shared_memory.SharedMemory(name=name, create=False)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass
        shm = multiprocessing.shared_memory.SharedMemory(name=name, create=True, size=size)
        for i in range(size):
            shm.buf[i] = 0
        return shm

    def _ensure_worker(self, config):
        """Start or restart the sensor worker process if needed."""
        if self._shm_accel is None:
            self._shm_accel = self._create_shm(SHM_NAME, SHM_SIZE)
            self._shm_gyro = self._create_shm(SHM_NAME_GYRO, SHM_SIZE)
            if config['ambient_light_enabled']:
                self._shm_als = self._create_shm(SHM_NAME_ALS, SHM_ALS_SIZE)
            if config['lid_angle_enabled']:
                self._shm_lid = self._create_shm(SHM_NAME_LID, SHM_LID_SIZE)

        if self._worker is None or not self._worker.is_alive():
            if self._worker is not None:
                self._restart_count += 1
                self.log.warning(
                    'Sensor worker died (restart #%d), restarting...',
                    self._restart_count,
                )

            kwargs = {
                'gyro_shm_name': SHM_NAME_GYRO,
            }
            if self._shm_als is not None:
                kwargs['als_shm_name'] = SHM_NAME_ALS
            if self._shm_lid is not None:
                kwargs['lid_shm_name'] = SHM_NAME_LID

            self._worker = multiprocessing.Process(
                target=sensor_worker,
                args=(SHM_NAME, self._restart_count),
                kwargs=kwargs,
                daemon=True,
            )
            self._worker.start()
            self.log.info('Sensor worker started (pid=%d)', self._worker.pid)

    # ------------------------------------------------------------------
    # Main check method
    # ------------------------------------------------------------------

    def check(self, instance):
        config = self._get_config(instance)
        tags = list(config.get('tags', []))

        # -- Platform gate --
        if not is_apple_silicon():
            self.service_check(
                SERVICE_CHECK, self.CRITICAL, tags=tags,
                message='Not running on Apple Silicon macOS',
            )
            return

        # -- Ensure sensor worker is running --
        try:
            self._ensure_worker(config)
        except Exception as e:
            self.service_check(
                SERVICE_CHECK, self.CRITICAL, tags=tags,
                message=f'Failed to start sensor worker: {e}',
            )
            self.log.error('Failed to start sensor worker: %s', e)
            return

        # -- Create detector on first run --
        if self._detector is None:
            self._detector = VibrationDetector(
                sensitivity=config['sensitivity'],
            )

        # -- Read accelerometer samples --
        try:
            samples, self._last_accel_total = shm_read_new(
                self._shm_accel.buf, self._last_accel_total,
            )
        except Exception as e:
            self.service_check(
                SERVICE_CHECK, self.CRITICAL, tags=tags,
                message=f'Failed to read accel shm: {e}',
            )
            return

        # -- Read gyroscope samples --
        gyro_samples = []
        try:
            gyro_samples, self._last_gyro_total = shm_read_new_gyro(
                self._shm_gyro.buf, self._last_gyro_total,
            )
        except Exception:
            pass  # gyro is optional for core metrics

        # -- Read ALS --
        if self._shm_als is not None:
            try:
                als_data, self._last_als_count = shm_snap_read(
                    self._shm_als.buf, self._last_als_count, ALS_REPORT_LEN,
                )
                if als_data is not None and len(als_data) >= _ALS_LUX_OFF + 4:
                    self._latest_als_lux = struct.unpack_from('<f', als_data, _ALS_LUX_OFF)[0]
            except Exception:
                pass

        # -- Read lid angle --
        if self._shm_lid is not None:
            try:
                lid_data, self._last_lid_count = shm_snap_read(
                    self._shm_lid.buf, self._last_lid_count, 4,
                )
                if lid_data is not None:
                    self._latest_lid_angle = struct.unpack('<f', lid_data)[0]
            except Exception:
                pass

        # -- Stale data check --
        if not samples:
            # No new data since last check
            if self._total_samples_seen == 0:
                self.service_check(
                    SERVICE_CHECK, self.WARNING, tags=tags,
                    message='No accelerometer data received yet (sensor may still be starting)',
                )
            else:
                self.service_check(
                    SERVICE_CHECK, self.WARNING, tags=tags,
                    message='No new accelerometer data in this interval',
                )
            return

        # -- Process samples through detector --
        now = time.time()
        if self._first_sample_time is None:
            self._first_sample_time = now

        # Reset per-interval stats before processing new batch
        self._detector.reset_interval()

        self._total_samples_seen += len(samples)

        # Feed gyro samples into detector (best-effort pairing)
        for gx, gy, gz in gyro_samples:
            self._detector.process_gyro(gx, gy, gz)

        # Feed accel samples and collect events
        events = []
        for ax, ay, az in samples:
            result = self._detector.process(ax, ay, az, now)
            if result is not None:
                events.append(result)

        # -- Emit acceleration gauges --
        latest = samples[-1]
        self.gauge('sentrymac.accel.x', latest[0], tags=tags)
        self.gauge('sentrymac.accel.y', latest[1], tags=tags)
        self.gauge('sentrymac.accel.z', latest[2], tags=tags)
        magnitude = math.sqrt(sum(v * v for v in latest))
        self.gauge('sentrymac.accel.magnitude', magnitude, tags=tags)

        # -- Emit vibration summary gauges --
        self.gauge('sentrymac.vibration.rms', self._detector.rms, tags=tags)
        self.gauge('sentrymac.vibration.peak', self._detector.peak, tags=tags)
        self.gauge('sentrymac.vibration.sta_lta_ratio', self._detector.sta_lta_latest, tags=tags)

        # -- Emit sample rate --
        elapsed = now - self._first_sample_time
        if elapsed > 1.0:
            rate = self._total_samples_seen / elapsed
            self.gauge('sentrymac.sensor.sample_rate', rate, tags=tags)

        # -- Emit orientation gauges --
        if config['orientation_enabled']:
            roll, pitch, yaw = self._detector.get_orientation_degrees()
            self.gauge('sentrymac.orientation.roll', roll, tags=tags)
            self.gauge('sentrymac.orientation.pitch', pitch, tags=tags)
            self.gauge('sentrymac.orientation.yaw', yaw, tags=tags)

            gx, gy, gz = self._detector.gyro_latest
            self.gauge('sentrymac.gyro.x', gx, tags=tags)
            self.gauge('sentrymac.gyro.y', gy, tags=tags)
            self.gauge('sentrymac.gyro.z', gz, tags=tags)

        # -- Emit lid angle --
        if config['lid_angle_enabled'] and self._latest_lid_angle is not None:
            self.gauge('sentrymac.lid.angle', self._latest_lid_angle, tags=tags)

        # -- Emit ambient light --
        if config['ambient_light_enabled'] and self._latest_als_lux is not None:
            self.gauge('sentrymac.ambient_light.lux', self._latest_als_lux, tags=tags)

        # -- Emit motion events --
        if config['motion_events_enabled']:
            min_sev_idx = SEVERITY_ORDER.index(
                config.get('motion_event_min_severity', 'vibration')
            ) if config.get('motion_event_min_severity', 'vibration') in SEVERITY_ORDER else 1

            for evt in events:
                sev_idx = SEVERITY_ORDER.index(evt['severity']) if evt['severity'] in SEVERITY_ORDER else 0
                if sev_idx < min_sev_idx:
                    continue
                self.event({
                    'timestamp': int(evt['timestamp']),
                    'event_type': 'sentrymac.motion',
                    'msg_title': f"SentryMac: {evt['severity_label']} detected",
                    'msg_text': (
                        f"Severity: {evt['severity_label']}\n"
                        f"Amplitude: {evt['amplitude']:.6f} g\n"
                        f"STA/LTA ratio: {evt['sta_lta_ratio']:.2f}\n"
                    ),
                    'alert_type': 'warning' if sev_idx <= 1 else 'error',
                    'tags': tags + [f"severity:{evt['severity']}"],
                })

        # -- Service check OK --
        self.service_check(
            SERVICE_CHECK, self.OK, tags=tags,
            message=f'{len(samples)} samples, {len(events)} events',
        )

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def cancel(self):
        """Called when the Agent stops the check. Terminate worker, unlink shm."""
        if self._worker is not None and self._worker.is_alive():
            self._worker.terminate()
            self._worker.join(timeout=3)
            if self._worker.is_alive():
                self._worker.kill()
            self.log.info('Sensor worker terminated')

        for shm in (self._shm_accel, self._shm_gyro, self._shm_als, self._shm_lid):
            if shm is not None:
                try:
                    shm.close()
                    shm.unlink()
                except Exception:
                    pass

        self._worker = None
        self._shm_accel = None
        self._shm_gyro = None
        self._shm_als = None
        self._shm_lid = None
