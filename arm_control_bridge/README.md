# arm_control_bridge

[English](./README.md) | [中文](./README_zh.md)

`arm_control_bridge` is the control bridge for a 4-axis arm plus claw channel:

- Upper layer → Bridge: TCP / HTTP commands
- Bridge → Raspberry Pi: UDP binary frames (v3, 140B, 25Hz)
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
  ├─ io/pi_controller.py → UDP 140B → Raspberry Pi                 │
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

- **Nosim**: enabled when `CONFIG.web_test_port > 0` (default `8877`); bind `CONFIG.web_test_host` (default `0.0.0.0`).
- **Sim**: same idea via `SIM_CONFIG.sim_web_port` / `SIM_CONFIG.sim_web_host`.
- On startup, the bridge logs a **browser-friendly URL** (LAN IP when bind is `0.0.0.0`, since `http://0.0.0.0:8877/` is not usable in browsers).
- All commands **block until reached** before returning (timeout from `CONFIG.reached_timeout_s` → 408).
- Routes: `POST /api/pose` · `/api/pose_delta` · `/api/joints` · `/api/joints_delta` · `/api/claw`

### 3) UDP to Pi

- Default target port: `9870` (`CONFIG.default_udp_port`, overridable with `--rpi-port`)
- Protocol: fixed v3 `140B` frame (`=Id + d*16`, 8-dim pos+vel), 25Hz
- See `doc/bridge2pi.md` for field details

### 4) Pi feedback (WebSocket)

- Subscribes to `ws://rpi_ip:8765` (pi2camera v1)
- Parses `feedback.fb_arm_rad` (preferred) or `feedback.mit_arm_rad`
- Started automatically when `--rpi-ip` is set (or `CONFIG.rpi_ip` default); auto-reconnect interval `CONFIG.pi_feedback_reconnect_interval_s`
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
- `io/pi_controller.py`: UDP v3 framing and `motor` / `servoMotor`
- `io/pi_feedback.py`: `PiFeedbackClient` (WebSocket)

### Kinematics and control

- `calculator.py`: stable public imports; implementations under `control/` and `kinematics/`
- `sim/shower.py`: `ArticulationViewer` / `FrameReceiver` for Isaac Sim

### Configuration and assets

- `config.py`: frozen dataclass defaults exposed as **`CONFIG`** (control / TCP / UDP / HTTP test / reached), **`IK_CONFIG`** (IK), **`SIM_CONFIG`** (Isaac assets + sim HTTP + PD), **`RUNTIME`** (e.g. `udp_strict`). Tweak fields there instead of growing the CLI.
- `configuration/`: URDF and USD assets (e.g. `configuration/v8/`)
- `web/`: test web UI (`index.html`)

### Documentation

- `doc/head2bridge.md`: upper-layer → bridge API spec (v2.2, includes reached reply)
- `doc/bridge2pi.md`: bridge → Pi UDP spec (v3)
- `doc/pi2camera.md`: Pi → camera/sim WebSocket broadcast spec

## CLI

Only network overrides on the command line; everything else is in `config.py`:

- `--sim` — Isaac Sim mode
- `--listen` — TCP bind host (default `CONFIG.listen_host`)
- `--port` — TCP JSON port (default `CONFIG.default_tcp_port`, usually `9888`)
- `--rpi-ip` — Pi IPv4 (default `CONFIG.rpi_ip`; omit UDP downlink if unset)
- `--rpi-port` — Pi UDP port (default `CONFIG.default_udp_port`, usually `9870`)

Headless sim, HTTP bind, IK tuning, USD paths: edit **`SIM_CONFIG`** / **`CONFIG`** / **`IK_CONFIG`**.

## Quick Start

```bash
# simulation + downlink to Pi (from clone root, e.g. icecreamarm/)
./arm_control_bridge/start_pc_control.sh sim 192.168.1.100

# open-loop downlink without simulation
./arm_control_bridge/start_pc_control.sh nosim 192.168.1.100

# local simulation only (no Pi); HTTP/sim flags in SIM_CONFIG
~/isaacsim/python.sh -m arm_control_bridge.run_control --sim

# same with explicit listen / Pi (optional)
~/isaacsim/python.sh -m arm_control_bridge.run_control --sim --listen 0.0.0.0 --port 9888 --rpi-ip 192.168.1.100
```

Run these from the repository root (the directory that contains the `arm_control_bridge/` package folder).

## Pi Integration

### Downlink (Bridge → Pi)

Bridge sends UDP 140B frames at `CONFIG.control_hz`. Set `--rpi-ip` (or default `CONFIG.rpi_ip`) to the Pi address. The HTTP test page also exposes `POST /api/stepper` and `POST /api/conveyor`.

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
