# apple-silicon-accelerometer

reading the undocumented mems accelerometer on apple silicon macbooks via iokit hid

![demo](assets/demo.gif)

## what is this

apple silicon chips (M1/M2/M3/M4) have a hidden mems accelerometer managed by the sensor processing unit (SPU).
it's not exposed through any public api or framework.
this project reads raw 3-axis acceleration data at ~800hz via iokit hid callbacks.

only tested on macbook pro m3 pro so far - might work on other apple silicon macs but no guarantees

## how it works

the sensor lives under AppleSPUHIDDevice in the iokit registry, on vendor usage page 0xFF00, usage 3.
the driver is AppleSPUHIDDriver which is part of the sensor processing unit.
we open it with IOHIDDeviceCreate and register an asynchronous callback via IOHIDDeviceRegisterInputReportCallback.
data comes as 22-byte hid reports with x/y/z as int32 little-endian at byte offsets 6, 10, 14.
divide by 65536 to get the value in g.
native callback rate is around 800hz, decimated to ~100hz for the analysis pipeline

you can verify the device exists on your machine with:

    ioreg -l -w0 | grep -A5 AppleSPUHIDDevice

## how to verify this is a real accelerometer

four tests you can do:

- slowly rotate your laptop 90 degrees - gravity should transfer smoothly between axes while |g| stays near 1.0
- hold it in the air with no surface contact - you should still read ~1g on the vertical axis
- place it on a cushion in a quiet room - the dc gravity reading stays stable even with zero mechanical input
- at rest |g| reads around 0.976 which is within normal mems tolerance (±2-5%)

if any of these fail it's not a real imu

## features

- 6-stage vibration detection pipeline
  - high-pass iir for gravity removal
  - sta/lta at 3 timescales (seismology-style event detection)
  - discrete wavelet transform (daubechies db4, 5 levels)
  - cusum bilateral for micro-change detection
  - kurtosis for transient impulsiveness
  - crest factor + mad for adaptive peak thresholding
- autocorrelation-based periodicity detection
- experimental heartbeat detection via ballistocardiography (bcg)
- real-time terminal dashboard with waveforms, spectrogram, event log

## quick start

    git clone https://github.com/olvvier/apple-silicon-accelerometer
    cd apple-silicon-accelerometer
    pip install -r requirements.txt
    sudo python3 motion_live.py

requires root because iokit hid device access on apple silicon needs elevated privileges

## code structure

- spu_sensor.py - the core: iokit bindings, device discovery, hid callback, shared memory ring buffer
- motion_live.py - vibration detection pipeline, heartbeat bcg, terminal ui, main loop

the sensor reading logic is isolated in spu_sensor.py so you can reuse it independently

## heartbeat demo

place your wrists on the laptop near the trackpad and wait 10-20 seconds for the signal to stabilize.
this uses ballistocardiography - the mechanical vibrations from your heartbeat transmitted through your arms into the chassis.
experimental, not reliable, just a fun use-case to show what the sensor can pick up.
the bcg bandpass is 0.8-3hz and bpm is estimated via autocorrelation on the filtered signal

## notes

- experimental / undocumented AppleSPU hid path
- requires sudo
- may break on future macos updates
- use at your own risk
- not for medical use

## tested on

- macbook pro m3 pro, macos 15.6.1
- python 3.14

## license

MIT

---

not affiliated with Apple or any employer
