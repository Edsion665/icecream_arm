# icecreamPi V2

Second-version runtime implementation for arm control.

## Modules

- `main.py`: compatibility entrypoint; delegates runtime to `app/runtime.py`
- `app/runtime.py`: lifecycle orchestration (startup/shutdown order, loops, component wiring)
- `serial.py`: STM32 serial thread manager (codec moved to infra layer)
- `controller.py`: MIT command generation, UDP stale handling, safety fallbacks
- `calculator.py`: gravity compensation calculation (includes Pinocchio gravity solver)
- `listener.py`: UDP packet listener and register refresh
- `server.py`: WebSocket state broadcast and servo command intake
- `state_store.py`: shared register storage
- `config.py`: environment-based config
- `domain/mapping.py`: centralized sign/axis mapping used by controller and calculator
- `domain/ports.py`: lightweight read/write/send interface boundaries
- `domain/command_handler.py`: WebSocket command semantic handling
- `infra/serial/codec.py`: MIT frame encode/decode pure functions
- `infra/udp/packet.py`: UDP packet format/unpack plus servo semantic decode helpers
- `infra/state/presenter.py`: state payload formatting for external consumers
- `robotarm/icecream_arm_model`: bundled URDF model assets used by gravity solver
- `start.sh`: repo-root launcher; `scripts/start.sh`: run logic (`.venv` first, then `uv run --project`)
- `requirements.txt`: runtime Python dependencies

## Layers

- `app`: runtime assembly and loop orchestration.
- `domain`: control/domain rules and interface contracts.
- `infra`: transport/codec/presenter implementation details.

## Class Purposes

### `config.py`
- `SerialConfig`: serial port parameters and defaults.
- `UdpConfig`: UDP listener and packet timing limits.
- `ServerConfig`: WebSocket host/port and push behavior.
- `ControlConfig`: arm control loop timing and safety thresholds.
- `AppConfig`: unified runtime config loader/validator.

### `controller.py`
- `ArmController`: core control loop; transforms state + command into MIT motor command and safety fallback.

### `app/runtime.py`
- `RuntimeComponents`: runtime dependency bundle.

### `listener.py`
- `UdpListener`: receives UDP packet, parses protocol fields, refreshes shared registers.

### `serial.py`
- `SerialManager`: serial worker thread; manages STM32 connection, RX/TX, and reconnect logic.

### `server.py`
- `StateServer`: WebSocket server for state broadcast and servo command intake.

### `domain/command_handler.py`
- `StateCommandHandler`: parses incoming WS command payloads and applies state mutations.

### `state_store.py`
- `UdpRegister`: latest UDP command/register snapshot.
- `FeedbackRegister`: latest motor/telemetry feedback snapshot.
- `RuntimeRegister`: runtime flags/state shared across workers.
- `StateStore`: central in-memory store with thread-safe read/write access.

### `calibration/collect.py`
- `InterpPoint`: calibration interpolation point model.
- `UdpCommandSender`: background thread that periodically sends UDP calibration command.
- `WsFeedbackBridge`: background thread that forwards WS feedback for calibration logging.

## Run

Dependencies are listed in `pyproject.toml` (`[project].dependencies`) and mirrored in `requirements.txt`. **Install [uv](https://github.com/astral-sh/uv)** and run `uv sync` at the repo root to maintain `.venv`.

`start.sh` delegates to `scripts/start.sh`, which `cd`s to the **parent of the repo** (so `python -m icecream.main` resolves the `icecream` package), then picks:

1. `icecream/.venv/bin/python -m icecream.main` if present  
2. else `uv run --project <repo-root> python -m icecream.main`  
3. else `python3` (you must install deps yourself)

**After clone or dependency changes:**

```bash
cd ~/icecream
uv sync
# or: uv pip install -r requirements.txt
```

**Start:**

```bash
cd ~/icecream
./start.sh
# or: bash scripts/start.sh
```

**Without uv**: from the repo parent directory, run `python3 -m icecream.main` after `pip install -r requirements.txt` (or equivalent).

Startup behavior note:
- Before starting serial/UDP loops, runtime performs a blocking Pinocchio gravity-model warmup.
- You will see startup logs:
  - `gravity model warmup start (pinocchio)`
  - `gravity model warmup done in ...s`
- This increases startup time once, but avoids first-use gravity initialization latency later.

## README Sync Requirement

For every code change, follow this workflow:

1. Read this `README.md` first to refresh project context.
2. Implement code changes.
3. Update this `README.md` if module/class behavior, architecture, protocol, or run steps changed.
4. Run the guard script before commit.

```bash
cd ~/icecream
bash scripts/guard_readme_sync.sh
```

The guard script blocks when Python code changed but `README.md` was not updated.

## Notes

- `icecreamPi` runtime no longer imports `arm_control` Python modules.
- Pi<->STM32 uses a 39-byte frame (`4 MIT + 2 servo + XOR`).
- Servo commands can come from UDP semantics and WS override, then are forwarded to STM32.
- bridge2pi uses v2.1 UDP 6D vectors (`108 bytes`).
- Protocol docs: `docs/bridge2pi.md`, `docs/pi2stm.md`, `docs/pi2camera.md`.
- WS `state.data.feedback` includes `link5_hmat` (`link0 -> link5`, 4x4 row-major, `null` when unavailable) and `link5_xyz_base_m` (`[x,y,z]` m, link5 origin in link0 frame, from the same transform).
- `link5_hmat` uses hybrid angle source: first 4 joints from feedback, joint5 from `udp.p_rel_deg[4]`.

