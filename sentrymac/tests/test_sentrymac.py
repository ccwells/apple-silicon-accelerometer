"""
Tests for the SentryMac Datadog integration.

These run without hardware — the sensor worker is mocked.
"""

import math
import struct
import time
import unittest
from unittest.mock import patch, MagicMock

from datadog_checks.sentrymac.detector import VibrationDetector, PRESETS
from datadog_checks.sentrymac.spu_sensor import (
    shm_write_sample, shm_read_new, shm_read_new_gyro,
    shm_snap_write, shm_snap_read,
    SHM_HEADER, RING_CAP, RING_ENTRY, SHM_SIZE,
    SHM_SNAP_HDR, ACCEL_SCALE, GYRO_SCALE,
    is_apple_silicon,
)


# ---------------------------------------------------------------------------
# Shared-memory ring buffer tests
# ---------------------------------------------------------------------------

class TestShmRingBuffer(unittest.TestCase):
    """Tests for the shared-memory ring buffer read/write helpers."""

    def _make_buf(self, size=SHM_SIZE):
        return bytearray(size)

    def test_write_and_read_single_sample(self):
        buf = self._make_buf()
        # Write one sample: (1.0g, 0.5g, -0.25g) as Q16 raw ints
        x_raw = int(1.0 * ACCEL_SCALE)
        y_raw = int(0.5 * ACCEL_SCALE)
        z_raw = int(-0.25 * ACCEL_SCALE)
        shm_write_sample(buf, x_raw, y_raw, z_raw)

        samples, total = shm_read_new(buf, 0)
        self.assertEqual(total, 1)
        self.assertEqual(len(samples), 1)
        self.assertAlmostEqual(samples[0][0], 1.0, places=4)
        self.assertAlmostEqual(samples[0][1], 0.5, places=4)
        self.assertAlmostEqual(samples[0][2], -0.25, places=4)

    def test_read_no_new_samples(self):
        buf = self._make_buf()
        samples, total = shm_read_new(buf, 0)
        self.assertEqual(len(samples), 0)
        self.assertEqual(total, 0)

    def test_write_and_read_multiple_samples(self):
        buf = self._make_buf()
        n = 50
        for i in range(n):
            x_raw = int(float(i) * ACCEL_SCALE)
            shm_write_sample(buf, x_raw, 0, 0)

        samples, total = shm_read_new(buf, 0)
        self.assertEqual(total, n)
        self.assertEqual(len(samples), n)
        # Verify order
        for i, (x, y, z) in enumerate(samples):
            self.assertAlmostEqual(x, float(i), places=2)

    def test_ring_buffer_wraparound(self):
        buf = self._make_buf()
        # Write more than RING_CAP samples — should wrap
        for i in range(RING_CAP + 100):
            shm_write_sample(buf, i, 0, 0)

        samples, total = shm_read_new(buf, 0)
        # Should only get RING_CAP samples (the latest ones)
        self.assertEqual(total, RING_CAP + 100)
        self.assertEqual(len(samples), RING_CAP)

    def test_incremental_reads(self):
        buf = self._make_buf()
        for i in range(10):
            shm_write_sample(buf, int(i * ACCEL_SCALE), 0, 0)

        samples1, total1 = shm_read_new(buf, 0)
        self.assertEqual(len(samples1), 10)

        for i in range(5):
            shm_write_sample(buf, int((10 + i) * ACCEL_SCALE), 0, 0)

        samples2, total2 = shm_read_new(buf, total1)
        self.assertEqual(len(samples2), 5)
        self.assertAlmostEqual(samples2[0][0], 10.0, places=2)


class TestShmSnapshot(unittest.TestCase):
    """Tests for the snapshot shared-memory helpers (ALS, lid angle)."""

    def test_snap_write_and_read(self):
        buf = bytearray(SHM_SNAP_HDR + 16)
        payload = b'\x01\x02\x03\x04'
        shm_snap_write(buf, payload)

        data, count = shm_snap_read(buf, 0, 4)
        self.assertIsNotNone(data)
        self.assertEqual(data, payload)
        self.assertEqual(count, 1)

    def test_snap_no_update(self):
        buf = bytearray(SHM_SNAP_HDR + 16)
        data, count = shm_snap_read(buf, 0, 4)
        self.assertIsNone(data)
        self.assertEqual(count, 0)


# ---------------------------------------------------------------------------
# Detector tests
# ---------------------------------------------------------------------------

class TestVibrationDetector(unittest.TestCase):
    """Tests for the simplified VibrationDetector."""

    def test_init_presets(self):
        for name in ('low', 'medium', 'high'):
            det = VibrationDetector(sensitivity=name)
            self.assertEqual(det.sta_lta_thresh, PRESETS[name]['sta_lta_thresh'])
            self.assertEqual(det.mag_floor, PRESETS[name]['mag_floor'])

    def test_static_input_no_events(self):
        """Constant gravity (~1g on Z) should produce no events."""
        det = VibrationDetector(sensitivity='medium', debounce_seconds=0.0)
        t = time.time()
        for i in range(500):
            result = det.process(0.0, 0.0, -1.0, t + i * 0.01)
            self.assertIsNone(result)

    def test_shock_triggers_event(self):
        """A sudden large impulse should trigger an event."""
        det = VibrationDetector(sensitivity='high', debounce_seconds=0.0)
        t = time.time()

        # Warm up with quiet samples
        for i in range(200):
            det.process(0.0, 0.0, -1.0, t + i * 0.01)

        # Inject a sharp impulse
        result = None
        for i in range(10):
            r = det.process(0.3, 0.3, -0.7, t + 2.0 + i * 0.01)
            if r is not None:
                result = r

        self.assertIsNotNone(result)
        self.assertIn(result['severity'], ('vibration', 'shock', 'major_shock'))
        self.assertGreater(result['amplitude'], 0)
        self.assertGreater(result['sta_lta_ratio'], 1.0)

    def test_debounce_suppresses_rapid_events(self):
        """Events within debounce window should be suppressed."""
        det = VibrationDetector(sensitivity='high', debounce_seconds=5.0)
        t = time.time()

        # Warm up
        for i in range(200):
            det.process(0.0, 0.0, -1.0, t + i * 0.01)

        events = []
        # Two impulses 1s apart (within 5s debounce)
        for burst_start in [2.0, 3.0]:
            for i in range(10):
                r = det.process(0.5, 0.5, -0.5, t + burst_start + i * 0.01)
                if r is not None:
                    events.append(r)
            # Return to quiet
            for i in range(50):
                det.process(0.0, 0.0, -1.0, t + burst_start + 0.1 + i * 0.01)

        # Should get at most 1 event due to debounce
        self.assertLessEqual(len(events), 1)

    def test_severity_classification(self):
        self.assertEqual(VibrationDetector._classify(0.1), ('major_shock', 'Major shock'))
        self.assertEqual(VibrationDetector._classify(0.03), ('shock', 'Shock'))
        self.assertEqual(VibrationDetector._classify(0.005), ('vibration', 'Vibration'))
        self.assertEqual(VibrationDetector._classify(0.001), ('micro_vib', 'Micro-vibration'))

    def test_rms_and_peak_tracking(self):
        det = VibrationDetector()
        t = time.time()
        # Process varying samples so the HP filter has a non-zero output
        for i in range(100):
            # Alternate vibration on X axis
            vib = 0.02 * (1 if i % 2 == 0 else -1)
            det.process(vib, 0.0, -1.0, t + i * 0.01)
        self.assertGreater(det.rms, 0)
        self.assertGreater(det.peak, 0)

        # Reset
        det.reset_interval()
        self.assertEqual(det.rms, 0.0)
        self.assertEqual(det.peak, 0.0)

    def test_orientation_degrees(self):
        det = VibrationDetector()
        t = time.time()
        # Process enough samples with gravity pointing down (-Z)
        for i in range(200):
            det.process(0.0, 0.0, -1.0, t + i * 0.01)
        roll, pitch, yaw = det.get_orientation_degrees()
        # With gravity on -Z and no rotation, roll and pitch should be near 0
        self.assertAlmostEqual(roll, 0.0, delta=10.0)
        self.assertAlmostEqual(pitch, 0.0, delta=10.0)

    def test_gyro_input(self):
        det = VibrationDetector()
        det.process_gyro(1.5, -2.3, 0.7)
        self.assertEqual(det.gyro_latest, (1.5, -2.3, 0.7))


# ---------------------------------------------------------------------------
# Platform check
# ---------------------------------------------------------------------------

class TestPlatformCheck(unittest.TestCase):

    @patch('datadog_checks.sentrymac.spu_sensor.sys')
    @patch('datadog_checks.sentrymac.spu_sensor.platform')
    def test_is_apple_silicon_true(self, mock_platform, mock_sys):
        mock_sys.platform = 'darwin'
        mock_platform.machine.return_value = 'arm64'
        self.assertTrue(is_apple_silicon())

    @patch('datadog_checks.sentrymac.spu_sensor.sys')
    @patch('datadog_checks.sentrymac.spu_sensor.platform')
    def test_is_apple_silicon_false_linux(self, mock_platform, mock_sys):
        mock_sys.platform = 'linux'
        mock_platform.machine.return_value = 'x86_64'
        self.assertFalse(is_apple_silicon())

    @patch('datadog_checks.sentrymac.spu_sensor.sys')
    @patch('datadog_checks.sentrymac.spu_sensor.platform')
    def test_is_apple_silicon_false_intel_mac(self, mock_platform, mock_sys):
        mock_sys.platform = 'darwin'
        mock_platform.machine.return_value = 'x86_64'
        self.assertFalse(is_apple_silicon())


# ---------------------------------------------------------------------------
# Check integration test (mocked sensor)
# ---------------------------------------------------------------------------

class TestSentryMacCheck(unittest.TestCase):
    """Integration test for SentryMacCheck with mocked sensor worker."""

    @patch('datadog_checks.sentrymac.check.is_apple_silicon', return_value=True)
    def test_check_emits_metrics_on_data(self, mock_platform):
        """Verify check emits expected metrics when samples are available."""
        from datadog_checks.sentrymac.check import SentryMacCheck
        from datadog_checks.sentrymac.spu_sensor import SHM_SIZE

        # Create check instance
        check = SentryMacCheck('sentrymac', {}, [{}])

        # Directly set internal shared-memory state (bypass _create_shm / worker)
        accel_buf = bytearray(SHM_SIZE)
        gyro_buf = bytearray(SHM_SIZE)

        mock_shm_accel = MagicMock()
        mock_shm_accel.buf = accel_buf
        mock_shm_gyro = MagicMock()
        mock_shm_gyro.buf = gyro_buf

        check._shm_accel = mock_shm_accel
        check._shm_gyro = mock_shm_gyro
        check._shm_als = None
        check._shm_lid = None

        # Fake worker that is "alive"
        mock_worker = MagicMock()
        mock_worker.is_alive.return_value = True
        check._worker = mock_worker

        # Write synthetic accel samples into the buffer
        for i in range(150):
            x_raw = int(0.001 * ACCEL_SCALE * (i % 3))
            y_raw = int(0.0005 * ACCEL_SCALE)
            z_raw = int(-1.0 * ACCEL_SCALE)
            shm_write_sample(accel_buf, x_raw, y_raw, z_raw)

        # Track emitted metrics
        emitted = {}
        def fake_gauge(name, value, tags=None):
            emitted[name] = value
        check.gauge = fake_gauge

        svc_checks = []
        def fake_sc(name, status, tags=None, message=''):
            svc_checks.append((name, status, message))
        check.service_check = fake_sc

        events_emitted = []
        check.event = lambda e: events_emitted.append(e)

        # Run check
        check.check({})

        # Verify accel metrics were emitted
        self.assertIn('sentrymac.accel.x', emitted)
        self.assertIn('sentrymac.accel.y', emitted)
        self.assertIn('sentrymac.accel.z', emitted)
        self.assertIn('sentrymac.accel.magnitude', emitted)
        self.assertIn('sentrymac.vibration.rms', emitted)
        self.assertIn('sentrymac.vibration.peak', emitted)
        self.assertIn('sentrymac.vibration.sta_lta_ratio', emitted)
        self.assertIn('sentrymac.orientation.roll', emitted)

        # Verify service check OK
        self.assertTrue(any(s[1] == 0 for s in svc_checks))  # OK = 0

    @patch('datadog_checks.sentrymac.check.is_apple_silicon', return_value=False)
    def test_check_critical_on_non_apple_silicon(self, mock_platform):
        from datadog_checks.sentrymac.check import SentryMacCheck

        check = SentryMacCheck('sentrymac', {}, [{}])

        svc_checks = []
        def fake_sc(name, status, tags=None, message=''):
            svc_checks.append((name, status, message))
        check.service_check = fake_sc

        check.check({})

        # Should emit CRITICAL service check
        self.assertTrue(any(s[1] == 2 for s in svc_checks))  # CRITICAL = 2


if __name__ == '__main__':
    unittest.main()
