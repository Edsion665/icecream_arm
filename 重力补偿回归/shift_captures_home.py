# One-shot: shift captures.jsonl from old HOME-relative coords to new HOME.
# r_new = r_old - calib where calib is FB at new mechanical zero (deg*100).
import json
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
PATH = HERE / "captures.jsonl"
CALIB = [8976, -233, 194, 83]  # J0..J3


def main():
    lines = PATH.read_text(encoding="utf-8").splitlines()
    out = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        o = json.loads(line)
        avg = o.get("avg", {})
        for i in range(4):
            k = f"motors.{i}.position"
            if k in avg and isinstance(avg[k], (int, float)):
                avg[k] = float(avg[k]) - CALIB[i]
        cmd = o.get("command", {})
        data = cmd.get("data")
        if isinstance(data, dict):
            for i, key in enumerate(["a0", "a1", "a2", "a3"]):
                if key in data:
                    data[key] = int(round(data[key] - CALIB[i]))
        o["avg"] = avg
        if data is not None:
            cmd["data"] = data
            o["command"] = cmd
        out.append(json.dumps(o, ensure_ascii=False))

    out_path = HERE / "captures_shifted_to_new_home.jsonl"
    out_path.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"Wrote {len(out)} records -> {out_path.name}")
    # Optional in-place replace (may fail if file locked by editor):
    try:
        backup = PATH.with_suffix(".jsonl.bak_before_home_shift")
        shutil.copy2(PATH, backup)
        PATH.write_text("\n".join(out) + "\n", encoding="utf-8")
        print(f"In-place OK; backup: {backup.name}")
    except OSError as e:
        print(f"In-place skipped ({e}); use {out_path.name} or close editor and re-run.")


if __name__ == "__main__":
    main()
