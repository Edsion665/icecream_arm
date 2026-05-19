# PC <-> RPi UDP Protocol (V3)

This document mirrors **`docs/bridge2pi.md` v3** for English readers and defines the PC-to-Raspberry-Pi UDP packet consumed by `icecreamPi`.

## Transport

- Protocol: UDP
- Default RPi bind: `0.0.0.0:9870`
- Config key: `ARM_CONTROL_RPI_UDP_PORT`

## Binary Layout

Fixed-size packet; Python struct:

`=Id` + `d` × **16**

- `=`: native endian, standard size, no alignment padding
- `I`: `seq` (`uint32`)
- `d`: `ts` (`float64`, sender timestamp, seconds)
- `d` × **8**: `p_rel_deg[8]` (`float64`, degrees / semantic floats per axis)
- `d` × **8**: `omega_rad_s[8]` (`float64`, rad/s)

**Total size: 140 bytes.**

Semantic indices:

| Index | `p_rel_deg` | `omega_rad_s` |
|---:|---|---|
| 0–3 | Arm joints J1–J4 (deg) | Joint velocities (rad/s) |
| 4 | `wrist_deg` | `0.0` |
| 5 | `grip_state` (`0=open`, `1=close` recommended) | `0.0` |
| 6 | `stepper_deg` incremental command (deg), same meaning as `docs/pi2stm.md` downlink | `0.0` |
| 7 | `conveyor_run` (`0.0` stop / `1.0` run) | `0.0` |

Pi clamps `[6]` to `[-180, 180]` (integer degrees after rounding) and binarizes `[7]` with the same threshold used for grip (`>= 0.5` → `1`), then forwards both into the **42-byte** MIT UART frame (`encode_mit_cmd_42`).

## Field Semantics

- `seq`: monotonic frame index (loss detection).
- `ts`: sender time (seconds); optional latency use.
- First four positions drive MIT motor targets after calibration mapping (unchanged).
- `p_rel_deg[4]` feeds FK / wrist servo path.
- `p_rel_deg[5]` is gripper discrete state (not FK).
- `p_rel_deg[6..7]` are forwarded to STM32 **in the same control cycle** as CAN + servos.

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

- UART Pi ↔ STM32 uses MIT **v3** (**42** bytes). See `docs/pi2stm.md`.
- Legacy **108-byte** UDP payloads are **rejected** by `icecreamPi/listener.py`.

## WS State Broadcast

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
      "p_rel_deg": [0.0, 10.0, -20.0, 5.0, 0.0, 1.0, 0.0, 0.0],
      "omega_rad_s": [0.0, 0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]
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
