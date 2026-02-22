# SentryMac — Datadog Agent Integration for MacBook Physical Security

Monitor the **physical state** of Apple Silicon MacBooks — acceleration, vibration, orientation, lid angle, and ambient light — as Datadog metrics and events. Enables fleet-wide physical security monitoring, tamper detection, and environmental dashboards.

## Overview

SentryMac reads the undocumented Bosch BMI286 IMU (accelerometer + gyroscope) on Apple Silicon MacBooks via IOKit HID. It packages this as a Datadog Agent custom check that continuously reports physical telemetry.

The Datadog Agent's macOS LaunchDaemon runs with root privileges by default, which means the IOKit HID sensor code runs directly inside the Agent — no privileged helper or Swift code required.

### Key Features

- **Acceleration metrics** — raw X/Y/Z, magnitude, vibration RMS and peak
- **Motion event detection** — STA/LTA classifier with severity levels (micro-vibration → major shock)
- **Orientation tracking** — roll/pitch/yaw via Mahony AHRS quaternion filter
- **Lid angle monitoring** — detect lid open/close events
- **Ambient light sensor** — track environmental light conditions
- **Pre-built dashboard** — fleet overview with all metrics
- **Recommended monitors** — "MacBook moved unexpectedly", "Sensor offline", "Lid opened after hours"

### Supported Hardware

- MacBook Pro M3 / M3 Pro / M3 Max (confirmed)
- MacBook Pro M5 (confirmed)
- Likely M2 Pro/Max and M4 series (untested)
- **Not supported**: Intel MacBooks, M1 MacBook Air/Pro (2020)

## Setup

### Prerequisites

- macOS on Apple Silicon (M2 Pro/Max or newer recommended)
- Datadog Agent 7.x installed via the standard macOS installer (LaunchDaemon, runs as root)

### Installation

Install the integration wheel:

```bash
sudo datadog-agent integration install --third-party datadog-sentrymac==0.1.0
```

Or build from source:

```bash
cd sentrymac
pip wheel --no-deps .
sudo datadog-agent integration install --third-party datadog_sentrymac-0.1.0-py3-none-any.whl
```

### Configuration

1. Copy the example config to the Agent's conf.d directory:

```bash
sudo cp /opt/datadog-agent/embedded/lib/python3.*/site-packages/datadog_checks/sentrymac/data/conf.yaml.example \
  /opt/datadog-agent/etc/conf.d/sentrymac.d/conf.yaml
```

2. Edit `/opt/datadog-agent/etc/conf.d/sentrymac.d/conf.yaml`:

```yaml
init_config:

instances:
  - min_collection_interval: 15
    sensitivity: medium
    motion_events_enabled: true
    motion_event_min_severity: vibration
    orientation_enabled: true
    ambient_light_enabled: true
    lid_angle_enabled: true
    tags:
      - team:security
      - location:hq-floor3
```

3. Restart the Agent:

```bash
sudo launchctl stop com.datadoghq.agent
sudo launchctl start com.datadoghq.agent
```

### Sensitivity Presets

| Preset | STA/LTA Threshold | Magnitude Floor | Use Case |
|--------|-------------------|-----------------|----------|
| `low` | > 5.0 | > 0.01 g | Only strong shocks/impacts |
| `medium` | > 3.0 | > 0.003 g | General tamper detection (default) |
| `high` | > 2.0 | > 0.001 g | Detect desk taps and light touches |

## Metrics

| Metric | Type | Description |
|--------|------|-------------|
| `sentrymac.accel.x/y/z` | gauge | Raw acceleration (g) |
| `sentrymac.accel.magnitude` | gauge | Total acceleration magnitude (g) |
| `sentrymac.vibration.rms` | gauge | Vibration RMS over check interval (g) |
| `sentrymac.vibration.peak` | gauge | Peak vibration in check interval (g) |
| `sentrymac.vibration.sta_lta_ratio` | gauge | STA/LTA energy ratio |
| `sentrymac.orientation.roll/pitch/yaw` | gauge | Device orientation (degrees) |
| `sentrymac.gyro.x/y/z` | gauge | Angular velocity (deg/s) |
| `sentrymac.lid.angle` | gauge | Lid angle (degrees) |
| `sentrymac.ambient_light.lux` | gauge | Ambient light (normalized 0-1) |
| `sentrymac.sensor.sample_rate` | gauge | Effective sample rate (Hz) |

## Service Checks

**sentrymac.sensor.status**
- `OK` — sensor producing data
- `WARNING` — no new data in this interval (stale)
- `CRITICAL` — SPU device not found, IOKit open failed, or not Apple Silicon

## Events

Motion events are emitted when the STA/LTA classifier triggers above the configured severity threshold:
- `micro_vib` — micro-vibration
- `vibration` — sustained vibration
- `shock` — impact/bump
- `major_shock` — strong impact or drop

## Dashboard

Import the included dashboard JSON (`assets/dashboard.json`) for a fleet-wide physical security overview including:
- Sensor health grid
- Motion event stream
- Vibration RMS/peak timeseries
- Orientation gauges
- Lid angle and ambient light sparklines

## Support

- **Source**: [github.com/olvvier/apple-silicon-accelerometer](https://github.com/olvvier/apple-silicon-accelerometer)
- **Issues**: File issues on the GitHub repository

## License

MIT
