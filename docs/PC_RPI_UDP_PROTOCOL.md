# PC <-> RPi UDP Protocol (V2)

This document defines the PC-to-Raspberry-Pi UDP packet used by `icecreamPi`.

## Transport

- Protocol: UDP
- Default RPi bind: `0.0.0.0:9870`
- Config key: `ARM_CONTROL_RPI_UDP_PORT`

## Binary Layout

The packet is fixed-size and uses Python struct format:

`=Idddddddddd`

- `=`: native endian, standard size, no alignment padding
- `I`: `seq` (`uint32`)
- `d`: `ts` (`float64`, sender monotonic/system timestamp, seconds)
- `d x 5`: `p_rel_deg[5]` (`float64[5]`, relative joint angles in degree)
- `d x 5`: `omega_rad_s[5]` (`float64[5]`, joint velocities in rad/s)

Total size: 92 bytes.

## Field Semantics

- `seq`
  - Monotonic incrementing frame index from PC sender.
  - Used for packet loss detection.
- `ts`
  - Sender timestamp in seconds.
  - Can be used for latency estimation (optional).
- `p_rel_deg`
  - Relative angle against calibration zero.
  - V2 consumes first 4 joints for motor command generation.
  - 5th value is `joint5_rel_deg` and is used for FK broadcast (`link5_hmat`).
- `omega_rad_s`
  - Joint angular velocities.
  - V2 consumes first 4 joints.
  - 5th value is reserved.

## Mapping in RPi Controller

Joint-to-motor mapping for first 4 axes:

- motor1 = `calibration[0] + (-1) * rad(p_rel_deg[0])`
- motor2 = `calibration[1] + (+1) * rad(p_rel_deg[1])`
- motor3 = `calibration[2] + (-1) * rad(p_rel_deg[2])`
- motor4 = `calibration[3] + (-1) * rad(p_rel_deg[3])`

Velocity follows the same sign mapping.

## Freshness and Timeout

- Freshness window: `ARM_CONTROL_RPI_UDP_STALE_SEC` (default `0.35s`).
- If packet age exceeds this value, controller enters hold behavior:
  - `p = current feedback angle`
  - `v = 0`
  - gravity compensation remains active when solver is healthy.

## Compatibility Notes

- STM32 serial protocol remains unchanged.
- This UDP protocol is PC<->RPi only.

## WS State Broadcast (V2)

In addition to UDP intake, RPi broadcasts runtime state over WebSocket:

- Default bind: `0.0.0.0:8765`
- Payload envelope:

```json
{
  "type": "state",
  "data": {
    "calibration_rad": [1.57, 1.45, 2.40, 0.49],
    "calibration_deg": [90.0, 83.1, 137.5, 28.1],
    "udp": {
      "seq": 1234,
      "age_ms": 18.7,
      "p_rel_deg": [0.0, 10.0, -20.0, 5.0, 0.0],
      "omega_rad_s": [0.0, 0.2, -0.3, 0.1, 0.0]
    },
    "feedback": {
      "mit_arm_rad": [1.57, 1.20, -0.40, 0.30],
      "fb_arm_rad": [1.57, 1.21, -0.41, 0.31],
      "link5_hmat": [
        [0.99, 0.01, 0.04, 0.28],
        [-0.01, 1.0, -0.02, 0.07],
        [-0.04, 0.02, 0.99, -0.01],
        [0.0, 0.0, 0.0, 1.0]
      ],
      "motors": [
        {"id": 0, "p": 1.57, "v": 0.0, "t": -0.02},
        {"id": 1, "p": 1.20, "v": 0.0, "t": 2.80},
        {"id": 2, "p": -0.40, "v": 0.0, "t": 3.70},
        {"id": 3, "p": 0.30, "v": 0.0, "t": 0.80}
      ],
      "crc_error_count": 0
    },
    "runtime": {
      "control_source": "udp",
      "safety_reason": "ok",
      "last_tau_nm": [0.0, 2.1, 3.5, 0.6],
      "servo_command": {}
    }
  }
}
```

### `feedback.link5_hmat` Definition

- Semantic: homogeneous transform matrix from `link0` frame to `link5` frame.
- Shape: `4x4` (row-major nested list).
- Unit:
  - rotation block `R` is unitless.
  - translation `[x, y, z]` is in meters.
- Source angle priority (hybrid strategy):
  1. `feedback.fb_arm_rad`
  2. fallback to `feedback.mit_arm_rad`
- joint5 source:
  - `udp.p_rel_deg[4]` (`joint5_rel_deg`, degree)
- Zero alignment:
  - first 4 joints use the same calibration/axis mapping as gravity compensation.
  - joint5 uses UDP relative degree by bridge2pi convention.
- Nullable behavior:
  - if feedback is unavailable, UDP has not been received, or FK computation fails, value is `null`.

