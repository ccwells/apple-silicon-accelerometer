"""
IOKit HID bindings for AppleSPUHIDDevice on Apple Silicon MacBooks.

Reads accelerometer (usage 3), gyroscope (usage 9), ambient light (usage 4),
and lid angle (usage 138) on vendor page 0xFF00.  Writes samples into POSIX
shared-memory ring buffers at ~100 Hz (decimated from ~800 Hz native).

Designed to run as a multiprocessing.Process target inside the Datadog Agent.
"""

import ctypes
import ctypes.util
import struct
import multiprocessing.shared_memory
import platform
import sys

# ---------------------------------------------------------------------------
# HID usage pages & usages
# ---------------------------------------------------------------------------
PAGE_VENDOR = 0xFF00            # Apple vendor page
PAGE_SENSOR = 0x0020            # HID sensor page
USAGE_ACCEL = 3                 # accelerometer
USAGE_GYRO = 9                  # gyroscope
USAGE_ALS = 4                   # ambient light
USAGE_LID = 138                 # lid angle sensor

# ---------------------------------------------------------------------------
# Report format (BMI286 IMU)
# ---------------------------------------------------------------------------
IMU_REPORT_LEN = 22             # accel/gyro report bytes
IMU_DECIMATION = 8              # keep 1 in N (~800 Hz -> ~100 Hz)
IMU_DATA_OFF = 6                # xyz payload start offset
ALS_REPORT_LEN = 122            # ALS report bytes
LID_REPORT_LEN = 3              # lid angle report bytes

# ---------------------------------------------------------------------------
# CoreFoundation type IDs
# ---------------------------------------------------------------------------
CF_UTF8 = 0x08000100            # kCFStringEncodingUTF8
CF_SINT32 = 3                   # kCFNumberSInt32Type
CF_SINT64 = 4                   # kCFNumberSInt64Type

# ---------------------------------------------------------------------------
# IOKit / driver constants
# ---------------------------------------------------------------------------
REPORT_BUF_SZ = 4096            # HID callback buffer
REPORT_INTERVAL_US = 1000       # driver report interval

# ---------------------------------------------------------------------------
# Scaling (Q16 raw -> physical units)
# ---------------------------------------------------------------------------
ACCEL_SCALE = 65536.0           # Q16 raw -> g
GYRO_SCALE = 65536.0            # Q16 raw -> deg/s

# ---------------------------------------------------------------------------
# Ring-buffer shared-memory layout
#   [0..3]   write_idx  u32
#   [4..11]  total      u64
#   [12..15] restarts   u32
#   [16..]   ring of RING_CAP * 12 bytes (3x i32: x, y, z)
# ---------------------------------------------------------------------------
RING_CAP = 8000
RING_ENTRY = 12
SHM_HEADER = 16
SHM_SIZE = SHM_HEADER + RING_CAP * RING_ENTRY

# Default shared-memory segment names (can be overridden via arguments)
SHM_NAME = 'sentrymac_accel'
SHM_NAME_GYRO = 'sentrymac_gyro'

# ---------------------------------------------------------------------------
# Snapshot shared-memory for low-rate sensors (latest value only)
#   [0..3]  update_count  u32
#   [4..7]  pad
#   [8..N]  payload
# ---------------------------------------------------------------------------
SHM_SNAP_HDR = 8
SHM_NAME_ALS = 'sentrymac_als'
SHM_NAME_LID = 'sentrymac_lid'
SHM_ALS_SIZE = SHM_SNAP_HDR + ALS_REPORT_LEN
SHM_LID_SIZE = SHM_SNAP_HDR + 4


# ---------------------------------------------------------------------------
# Shared-memory read/write helpers
# ---------------------------------------------------------------------------

def shm_write_sample(buf, x_raw, y_raw, z_raw):
    """Write one (x, y, z) int32 sample into the ring buffer."""
    idx, = struct.unpack_from('<I', buf, 0)
    off = SHM_HEADER + idx * RING_ENTRY
    struct.pack_into('<iii', buf, off, x_raw, y_raw, z_raw)
    struct.pack_into('<I', buf, 0, (idx + 1) % RING_CAP)
    total, = struct.unpack_from('<Q', buf, 4)
    struct.pack_into('<Q', buf, 4, total + 1)


def shm_read_new(buf, last_total):
    """Read new accel samples since *last_total*.  Returns (list_of_g_tuples, new_total)."""
    total, = struct.unpack_from('<Q', buf, 4)
    n_new = total - last_total
    if n_new <= 0:
        return [], total
    if n_new > RING_CAP:
        n_new = RING_CAP
    idx, = struct.unpack_from('<I', buf, 0)
    samples = []
    start = (idx - n_new) % RING_CAP
    for i in range(n_new):
        pos = (start + i) % RING_CAP
        off = SHM_HEADER + pos * RING_ENTRY
        x, y, z = struct.unpack_from('<iii', buf, off)
        samples.append((x / ACCEL_SCALE, y / ACCEL_SCALE, z / ACCEL_SCALE))
    return samples, total


def shm_read_new_gyro(buf, last_total):
    """Read new gyro samples since *last_total*.  Returns (list_of_dps_tuples, new_total)."""
    total, = struct.unpack_from('<Q', buf, 4)
    n_new = total - last_total
    if n_new <= 0:
        return [], total
    if n_new > RING_CAP:
        n_new = RING_CAP
    idx, = struct.unpack_from('<I', buf, 0)
    samples = []
    start = (idx - n_new) % RING_CAP
    for i in range(n_new):
        pos = (start + i) % RING_CAP
        off = SHM_HEADER + pos * RING_ENTRY
        x, y, z = struct.unpack_from('<iii', buf, off)
        samples.append((x / GYRO_SCALE, y / GYRO_SCALE, z / GYRO_SCALE))
    return samples, total


def shm_snap_write(buf, payload):
    """Write a snapshot payload and bump the update counter."""
    buf[SHM_SNAP_HDR:SHM_SNAP_HDR + len(payload)] = payload
    cnt, = struct.unpack_from('<I', buf, 0)
    struct.pack_into('<I', buf, 0, cnt + 1)


def shm_snap_read(buf, last_count, payload_len):
    """Read the latest snapshot if updated.  Returns (bytes_or_None, new_count)."""
    cnt, = struct.unpack_from('<I', buf, 0)
    if cnt == last_count:
        return None, cnt
    return bytes(buf[SHM_SNAP_HDR:SHM_SNAP_HDR + payload_len]), cnt


# ---------------------------------------------------------------------------
# Platform check
# ---------------------------------------------------------------------------

def is_apple_silicon():
    """Return True if running on an Apple Silicon Mac."""
    return sys.platform == 'darwin' and platform.machine() == 'arm64'


# ---------------------------------------------------------------------------
# Sensor worker — runs as multiprocessing.Process target
# ---------------------------------------------------------------------------

def sensor_worker(shm_name, restart_count, gyro_shm_name=None,
                  als_shm_name=None, lid_shm_name=None):
    """
    Main loop for the sensor reader process.

    Opens IOKit HID devices, registers callbacks, and spins a CFRunLoop.
    Writes samples into the pre-created shared-memory segments identified
    by *shm_name* (accel) and optional *gyro_shm_name*, *als_shm_name*,
    *lid_shm_name*.
    """
    _iokit = ctypes.cdll.LoadLibrary(ctypes.util.find_library('IOKit'))
    _cf = ctypes.cdll.LoadLibrary(ctypes.util.find_library('CoreFoundation'))

    kCFAllocatorDefault = ctypes.c_void_p.in_dll(_cf, 'kCFAllocatorDefault')
    kCFRunLoopDefaultMode = ctypes.c_void_p.in_dll(_cf, 'kCFRunLoopDefaultMode')

    _iokit.IOServiceMatching.restype = ctypes.c_void_p
    _iokit.IOServiceMatching.argtypes = [ctypes.c_char_p]
    _iokit.IOServiceGetMatchingServices.restype = ctypes.c_int
    _iokit.IOServiceGetMatchingServices.argtypes = [
        ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)]
    _iokit.IOIteratorNext.restype = ctypes.c_uint
    _iokit.IOIteratorNext.argtypes = [ctypes.c_uint]
    _iokit.IOObjectRelease.argtypes = [ctypes.c_uint]
    _iokit.IORegistryEntryCreateCFProperty.restype = ctypes.c_void_p
    _iokit.IORegistryEntryCreateCFProperty.argtypes = [
        ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint]
    _iokit.IORegistryEntrySetCFProperty.restype = ctypes.c_int
    _iokit.IORegistryEntrySetCFProperty.argtypes = [
        ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p]
    _iokit.IOHIDDeviceCreate.restype = ctypes.c_void_p
    _iokit.IOHIDDeviceCreate.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    _iokit.IOHIDDeviceOpen.restype = ctypes.c_int
    _iokit.IOHIDDeviceOpen.argtypes = [ctypes.c_void_p, ctypes.c_int]
    _iokit.IOHIDDeviceRegisterInputReportCallback.restype = None
    _iokit.IOHIDDeviceRegisterInputReportCallback.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_long,
        ctypes.c_void_p, ctypes.c_void_p]
    _iokit.IOHIDDeviceScheduleWithRunLoop.restype = None
    _iokit.IOHIDDeviceScheduleWithRunLoop.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]

    _cf.CFStringCreateWithCString.restype = ctypes.c_void_p
    _cf.CFStringCreateWithCString.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32]
    _cf.CFNumberCreate.restype = ctypes.c_void_p
    _cf.CFNumberCreate.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    _cf.CFNumberGetValue.restype = ctypes.c_bool
    _cf.CFNumberGetValue.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    _cf.CFRunLoopGetCurrent.restype = ctypes.c_void_p
    _cf.CFRunLoopRunInMode.restype = ctypes.c_int32
    _cf.CFRunLoopRunInMode.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_bool]

    def cfstr(s):
        return _cf.CFStringCreateWithCString(None, s.encode(), CF_UTF8)

    def cfnum32(v):
        val = ctypes.c_int32(v)
        return _cf.CFNumberCreate(None, CF_SINT32, ctypes.byref(val))

    def prop_int(svc, k):
        ref = _iokit.IORegistryEntryCreateCFProperty(svc, cfstr(k), None, 0)
        if not ref:
            return None
        v = ctypes.c_long()
        _cf.CFNumberGetValue(ref, CF_SINT64, ctypes.byref(v))
        return v.value

    # -- attach to shared-memory segments --
    accel_shm = multiprocessing.shared_memory.SharedMemory(name=shm_name, create=False)
    accel_buf = accel_shm.buf
    struct.pack_into('<I', accel_buf, 12, restart_count)

    gyro_buf = None
    if gyro_shm_name:
        gyro_shm = multiprocessing.shared_memory.SharedMemory(
            name=gyro_shm_name, create=False)
        gyro_buf = gyro_shm.buf

    als_buf = None
    if als_shm_name:
        als_shm = multiprocessing.shared_memory.SharedMemory(
            name=als_shm_name, create=False)
        als_buf = als_shm.buf

    lid_buf = None
    if lid_shm_name:
        lid_shm = multiprocessing.shared_memory.SharedMemory(
            name=lid_shm_name, create=False)
        lid_buf = lid_shm.buf

    # -- callback definitions --
    _REPORT_CB = ctypes.CFUNCTYPE(
        None, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p,
        ctypes.c_int, ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_long,
    )

    accel_dec = [0]

    def on_accel_report(ctx, result, sender, rtype, rid, rpt, length):
        try:
            if length == IMU_REPORT_LEN:
                accel_dec[0] += 1
                if accel_dec[0] < IMU_DECIMATION:
                    return
                accel_dec[0] = 0
                data = bytes(rpt[:IMU_REPORT_LEN])
                o = IMU_DATA_OFF
                x = struct.unpack('<i', data[o:o + 4])[0]
                y = struct.unpack('<i', data[o + 4:o + 8])[0]
                z = struct.unpack('<i', data[o + 8:o + 12])[0]
                shm_write_sample(accel_buf, x, y, z)
        except Exception:
            pass

    accel_cb_ref = _REPORT_CB(on_accel_report)

    gyro_dec = [0]
    gyro_cb_ref = None

    if gyro_buf is not None:
        def on_gyro_report(ctx, result, sender, rtype, rid, rpt, length):
            try:
                if length == IMU_REPORT_LEN:
                    gyro_dec[0] += 1
                    if gyro_dec[0] < IMU_DECIMATION:
                        return
                    gyro_dec[0] = 0
                    data = bytes(rpt[:IMU_REPORT_LEN])
                    o = IMU_DATA_OFF
                    x = struct.unpack('<i', data[o:o + 4])[0]
                    y = struct.unpack('<i', data[o + 4:o + 8])[0]
                    z = struct.unpack('<i', data[o + 8:o + 12])[0]
                    shm_write_sample(gyro_buf, x, y, z)
            except Exception:
                pass

        gyro_cb_ref = _REPORT_CB(on_gyro_report)

    als_cb_ref = None
    if als_buf is not None:
        def on_als_report(ctx, result, sender, rtype, rid, rpt, length):
            try:
                if length == ALS_REPORT_LEN:
                    shm_snap_write(als_buf, bytes(rpt[:ALS_REPORT_LEN]))
            except Exception:
                pass

        als_cb_ref = _REPORT_CB(on_als_report)

    lid_cb_ref = None
    if lid_buf is not None:
        def on_lid_report(ctx, result, sender, rtype, rid, rpt, length):
            try:
                if length >= LID_REPORT_LEN:
                    data = bytes(rpt[:length])
                    if data[0] != 1:
                        return
                    angle = struct.unpack('<H', data[1:3])[0] & 0x1FF
                    struct.pack_into('<f', lid_buf, SHM_SNAP_HDR, float(angle))
                    cnt, = struct.unpack_from('<I', lid_buf, 0)
                    struct.pack_into('<I', lid_buf, 0, cnt + 1)
            except Exception:
                pass

        lid_cb_ref = _REPORT_CB(on_lid_report)

    # -- wake the SPU drivers --
    matching = _iokit.IOServiceMatching(b'AppleSPUHIDDriver')
    it = ctypes.c_uint()
    _iokit.IOServiceGetMatchingServices(0, matching, ctypes.byref(it))
    while True:
        svc = _iokit.IOIteratorNext(it.value)
        if not svc:
            break
        for k, v in [('SensorPropertyReportingState', 1),
                     ('SensorPropertyPowerState', 1),
                     ('ReportInterval', REPORT_INTERVAL_US)]:
            _iokit.IORegistryEntrySetCFProperty(svc, cfstr(k), cfnum32(v))
        _iokit.IOObjectRelease(svc)

    # -- open HID devices and register callbacks --
    gc_roots = []

    matching = _iokit.IOServiceMatching(b'AppleSPUHIDDevice')
    it2 = ctypes.c_uint()
    _iokit.IOServiceGetMatchingServices(0, matching, ctypes.byref(it2))
    while True:
        svc = _iokit.IOIteratorNext(it2.value)
        if not svc:
            break
        up = prop_int(svc, 'PrimaryUsagePage') or 0
        u = prop_int(svc, 'PrimaryUsage') or 0

        cb = None
        if (up, u) == (PAGE_VENDOR, USAGE_ACCEL):
            cb = accel_cb_ref
        elif (up, u) == (PAGE_VENDOR, USAGE_GYRO) and gyro_cb_ref is not None:
            cb = gyro_cb_ref
        elif (up, u) == (PAGE_VENDOR, USAGE_ALS) and als_cb_ref is not None:
            cb = als_cb_ref
        elif (up, u) == (PAGE_SENSOR, USAGE_LID) and lid_cb_ref is not None:
            cb = lid_cb_ref

        if cb is not None:
            hid = _iokit.IOHIDDeviceCreate(kCFAllocatorDefault, svc)
            if hid:
                kr = _iokit.IOHIDDeviceOpen(hid, 0)
                if kr == 0:
                    rb = (ctypes.c_uint8 * REPORT_BUF_SZ)()
                    gc_roots.append(rb)
                    _iokit.IOHIDDeviceRegisterInputReportCallback(
                        hid, rb, REPORT_BUF_SZ, cb, None)
                    _iokit.IOHIDDeviceScheduleWithRunLoop(
                        hid, _cf.CFRunLoopGetCurrent(), kCFRunLoopDefaultMode)
        _iokit.IOObjectRelease(svc)

    gc_roots.extend([accel_cb_ref, gyro_cb_ref, als_cb_ref, lid_cb_ref])

    # -- run the CFRunLoop forever --
    while True:
        _cf.CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, False)
