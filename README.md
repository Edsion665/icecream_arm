# arm_control_bridge

`arm_control_bridge` is the control bridge for a 4-axis arm plus claw channel:

- Upper layer -> Bridge: TCP/HTTP commands
- Bridge -> Raspberry Pi: UDP binary frames
- Optional Isaac Sim runtime (`--sim`)

## Architecture

Upper layer sends commands (`pose/joints/claw`) -> `listener.py` normalizes them ->
`calculator.py` generates joint targets -> `PiController.py` sends V2.1 UDP frames ->
Pi side executes motor/servo actions. In simulation mode, the same command stream also drives Isaac Sim.

## Interfaces

### 1) TCP (JSON lines)

- Default listen: `0.0.0.0:9888`
- One JSON object per line (`\n`)
- Main options: `--listen`, `--port`

### 2) HTTP (JSON)

- Enabled when `--web-port > 0`
- Default from `run_control.py`: `127.0.0.1:8765`
- Script default from `start_pc_control.sh`: `0.0.0.0:8877`
- Routes:
  - `POST /api/pose`
  - `POST /api/pose_delta`
  - `POST /api/joints`
  - `POST /api/joints_delta`
  - `POST /api/claw`

### 3) UDP to Pi

- Default target port: `9870` (`--rpi-port`)
- Protocol: V2.1 fixed `108B` frame (`=Id + d*12`)
- Default control loop: `25Hz`

For detailed protocol fields, see `doc/head2bridge.md` and `doc/bridge2pi.md`.

## Command Schema (High-frequency summary)

- `pose`: required `x, y, z` (meters)
- `pose_delta`: required `dx, dy, dz` (meters)
- `joints`: required `axes_rel_deg` (array length 4)
- `joints_delta`: required `deltas_rel_deg` (array length 4)
- `claw`: required `wrist_deg` + (`grip_state` or `open_close`)
- `stop` / `ping`: accepted by TCP command path

Minimal TCP example:

```json
{"cmd":"joints","axes_rel_deg":[0,10,-90,-70]}
{"cmd":"claw","wrist_deg":20,"open_close":"close"}
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
```

## File & Folder Responsibilities

### Core runtime

- `run_control.py`: main entrypoint, CLI args, sim/nosim loop orchestration
- `start_pc_control.sh`: one-click launcher (sim or nosim)

### Command ingress and protocol I/O

- `listener.py`: TCP/HTTP server and command normalization
- `PiController.py`: UDP packet packing and downstream sending

### Kinematics and control

- `calculator.py`: IK/FK, command application, frame generation
- `shower.py`: simulation-side visualization/state display helpers

### Configuration and assets

- `config.py`: default ports, rates, limits, calibration loading
- `configuration/`: runtime assets (e.g., default URDF files)
- `web/`: test web UI (`index.html`)

### Documentation

- `doc/head2bridge.md`: upper-layer -> bridge API spec
- `doc/bridge2pi.md`: bridge -> Pi UDP spec
- `doc/pi2camera.md`: camera-side related protocol note


### One-click script

```bash
# simulation + downlink
./arm_control_bridge/start_pc_control.sh sim 192.168.1.100

# open-loop downlink without simulation
./arm_control_bridge/start_pc_control.sh nosim 192.168.1.100
```

## Minimal Integration

```bash
python3 - <<'PY'
import socket, json
s = socket.create_connection(("127.0.0.1", 9888))
for cmd in [
    {"cmd":"pose","x":0.35,"y":0.2,"z":0.25},
    {"cmd":"joints","axes_rel_deg":[0,0,5,0]},
    {"cmd":"claw","wrist_deg":15,"grip":0.5},
]:
    s.sendall((json.dumps(cmd) + "\n").encode("utf-8"))
s.close()
print("done")
PY
```
