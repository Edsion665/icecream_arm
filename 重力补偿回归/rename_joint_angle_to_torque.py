"""Rename avg keys joints.N.angle -> joints.N.torque (values are N·m per FB line j1..j4)."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
FILES = [
    "captures.jsonl",
    "captures_shifted_to_new_home.jsonl",
    "captures.jsonl.bak_before_home_shift",
]
RENAME = {f"joints.{i}.angle": f"joints.{i}.torque" for i in range(1, 5)}


def rewrite(path: Path) -> int:
    if not path.is_file():
        return 0
    lines = path.read_text(encoding="utf-8").splitlines()
    out = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        o = json.loads(line)
        avg = o.get("avg")
        if isinstance(avg, dict):
            new_avg = {}
            for k, v in avg.items():
                new_avg[RENAME.get(k, k)] = v
            o["avg"] = new_avg
        out.append(json.dumps(o, ensure_ascii=False))
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
    return len(out)


def main():
    for name in FILES:
        p = HERE / name
        try:
            n = rewrite(p)
            if n:
                print(f"{name}: {n} lines")
        except OSError as e:
            print(f"{name}: skipped ({e})")


if __name__ == "__main__":
    main()
