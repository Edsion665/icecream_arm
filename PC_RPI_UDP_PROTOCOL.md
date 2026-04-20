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
  - 5th value is reserved.
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

