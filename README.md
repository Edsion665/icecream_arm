# arm_control_bridge

[English](./README.md) | [中文](./README_zh.md)

`arm_control_bridge` is the control bridge for a 4-axis arm plus claw channel:

- Upper layer → Bridge: TCP / HTTP commands
- Bridge → Raspberry Pi: UDP binary frames (V2.1, 108B, 25Hz)
- Bridge ← Raspberry Pi: WebSocket state broadcast (pi2camera v1, used for reached detection)
- Optional Isaac Sim runtime (`--sim`)

## Architecture

```
Upper-layer policy
  │  TCP JSON lines / HTTP POST
  ▼
io/listener.py  ───────────────────────────────────────────────────
  CommandNormalizer normalizes commands                             │
  register_pending(tcp_conn / http_slot)  ← pre-register callback  │
  │                                                                 │
  ▼                                                                 │
run_control.py control loop (25Hz)                                  │
  runtime/reach_tracker  ← reached tracking + async reply queue     │
  engine.apply_command() ← update IK / joint target                │
  engine.step()          ← generate JointFrame                     │
  tracker.feed(reached, result)                                    │
  │                                                                 │
  ├─ io/pi_controller.py → UDP 108B → Raspberry Pi                 │
  │                                                                 │
  └─ reply_q (daemon thread)                                        │
       conn.sendall / slot.event.set  ← async reply, non-blocking  │
                                                                    │
io/pi_feedback.py (daemon thread)                                    │
  WebSocket subscribe ws://rpi_ip:8765                              │
  parse feedback.fb_arm_rad / mit_arm_rad                          │
  get_fb_arm_rad() → real joint angles for is_reached() ───────────┘
```

**Reached detection source priority**: real joint angles from Pi WebSocket when available; falls back to `state.q_cmd` (or `arm.get_joint_positions()` in sim mode).

## Interfaces

### 1) TCP (JSON lines)

- Default listen: `0.0.0.0:9888`
- One JSON object per line (`\n`)
- Bridge writes back one JSON line on the **same connection** when reached (see §Reached Reply)

### 2) HTTP (JSON)

- Enabled when `--web-port > 0`; `start_pc_control.sh` default: `0.0.0.0:8877`
- All commands **block until reached** before returning (timeout 10s → 408)
- Routes: `POST /api/pose` · `/api/pose_delta` · `/api/joints` · `/api/joints_delta` · `/api/claw`

### 3) UDP to Pi

- Default target port: `9870` (`--rpi-port`)
- Protocol: fixed V2.1 `108B` frame (`=Id + d*12`), 25Hz
- See `doc/bridge2pi.md` for field details

### 4) Pi feedback (WebSocket)

- Subscribes to `ws://rpi_ip:8765` (pi2camera v1)
- Parses `feedback.fb_arm_rad` (preferred) or `feedback.mit_arm_rad`
- Started automatically when `--rpi-ip` is set; auto-reconnects on disconnect (3s)
- Gracefully degrades to internal command angles when no feedback is available

## Reached Reply Format (head2bridge v2.2)

After a command completes, bridge sends the result back to the caller.

**joints / joints_delta reached:**
```json
{"ok": true, "reached": true, "error_joints_deg": 1.2}
```

**pose / pose_delta reached:**
```json
{"ok": true, "reached": true, "actual_pose": {"x": 0.349, "y": 0.201, "z": 0.248}, "error_pose_m": 0.003}
```

**claw reached (2s timer):**
```json
{"ok": true, "reached": true}
```

**Timeout (10s):**
```json
{"ok": false, "reached": false, "error": "timeout"}
```

Reached detection rules:
- `joints`: every axis error < 5° (any axis over threshold → not reached)
- `pose`: link4 FK position error norm < 5mm
- `claw`: no hardware feedback; auto-reached 2s after command received
- Stability buffer: 5 consecutive frames (@25Hz ≈ 200ms) must all satisfy the threshold before triggering reply — prevents transient false positives

## Layout (by concern)

| Path | Role |
|------|------|
| `run_control.py` | CLI entry and `run_loop` / `run_sim_loop` orchestration |
| `config.py`, `exceptions.py` | Global settings and bridge-specific errors |
| `calculator.py` | Re-exports `control/` and `kinematics/` for stable imports |
| `control/`, `kinematics/` | State, IK, URDF kinematics |
| `io/` | TCP/HTTP (`listener.py`), Pi UDP (`pi_controller.py`), WS feedback (`pi_feedback.py`) |
| `sim/` | Isaac scene helpers (`bootstrap.py`), articulation viewer (`shower.py`) |
| `runtime/` | Reached tracking / reply thread (`reach_tracker.py`), UDP frame debug (`udp_debug.py`) |
| `PiController.py` | Compatibility shim forwarding to `io.pi_controller` |

### Command ingress and protocol I/O (`io/`)

- `io/listener.py`: TCP/HTTP, `CommandNormalizer`, `ReplySlot`, `on_pending`
- `io/pi_controller.py`: UDP V2.1 framing and `motor` / `servoMotor`
- `io/pi_feedback.py`: `PiFeedbackClient` (WebSocket)

### Kinematics and control

- `calculator.py`: stable public imports; implementations under `control/` and `kinematics/`
- `sim/shower.py`: `ArticulationViewer` / `FrameReceiver` for Isaac Sim

### Configuration and assets

- `config.py`: ports, rates, limits, calibration loading, and reached thresholds (`REACHED_JOINTS_TOL_DEG=5°`, `REACHED_POSE_TOL_M=5mm`, `REACHED_STABLE_FRAMES=5`, `REACHED_TIMEOUT_S=10s`, `REACHED_CLAW_DELAY_S=2s`)
- `configuration/`: URDF and USD assets
- `web/`: test web UI (`index.html`)

### Documentation

- `doc/head2bridge.md`: upper-layer → bridge API spec (v2.2, includes reached reply)
- `doc/bridge2pi.md`: bridge → Pi UDP spec (V2.1)
- `doc/pi2camera.md`: Pi → camera/sim WebSocket broadcast spec

## Quick Start

```bash
# simulation + downlink to Pi
./arm_control_bridge/start_pc_control.sh sim 192.168.1.100

# open-loop downlink without simulation
./arm_control_bridge/start_pc_control.sh nosim 192.168.1.100

# local simulation only (no Pi)
~/isaacsim/python.sh -m arm_control_bridge.run_control --sim --web-port 8877 --web-host 0.0.0.0

# headless simulation (lower GPU load)
~/isaacsim/python.sh -m arm_control_bridge.run_control --sim --headless --web-port 8877 --web-host 0.0.0.0
```

## Pi Integration

### Downlink (Bridge → Pi)

Bridge sends UDP 108B frames at 25Hz. Set `--rpi-ip` to the Pi address; no other configuration needed.

### Uplink (Pi → Bridge, for reached detection)

The Pi runs `icecreamPi`, which broadcasts state over WebSocket (`ws://pi_ip:8765`, ~20Hz). When `--rpi-ip` is set, bridge automatically starts `PiFeedbackClient` to subscribe.

Pi broadcast message format (pi2camera v1):
```json
{
  "type": "state",
  "data": {
    "feedback": {
      "fb_arm_rad": [q1, q2, q3, q4],
      "mit_arm_rad": [q1, q2, q3, q4]
    }
  }
}
```

Bridge prefers `fb_arm_rad`, falls back to `mit_arm_rad`, and degrades to internal command angles if neither is present.

### Integration Verification

```bash
# send joints command, wait for reached reply
curl -s -X POST http://127.0.0.1:8877/api/joints \
  -H "Content-Type: application/json" \
  -d '{"axes_rel_deg":[0,90,-180,-20]}'
# returns: {"ok":true,"reached":true,"error_joints_deg":0.8}

# send pose command
curl -s -X POST http://127.0.0.1:8877/api/pose \
  -H "Content-Type: application/json" \
  -d '{"x":-0.05,"y":0.05,"z":0.28}'
# returns: {"ok":true,"reached":true,"actual_pose":{"x":-0.05,"y":0.05,"z":0.28},"error_pose_m":0.002}

# TCP path (reply arrives on the same connection)
python3 -c "
import socket, json
s = socket.create_connection(('127.0.0.1', 9888))
s.sendall((json.dumps({'cmd':'joints','axes_rel_deg':[0,90,-180,-20]}) + '\n').encode())
s.settimeout(15)
print('reply:', s.recv(4096).decode())
s.close()
"
```

## Command Schema

| Command | Required fields | Notes |
|---|---|---|
| `pose` | `x, y, z` (m) | Cartesian target for link4; blocks until reached |
| `pose_delta` | `dx, dy, dz` (m) | Cartesian increment; blocks until reached |
| `joints` | `axes_rel_deg` (length 4) | Absolute joint target relative to calibration zero (deg); blocks until reached |
| `joints_delta` | `deltas_rel_deg` (length 4) | Joint increment; blocks until reached |
| `claw` | `wrist_deg` + `grip_state`/`open_close` | Wrist angle + gripper state; replies after 2s |
| `stop` / `ping` | — | E-stop log / connectivity check |
