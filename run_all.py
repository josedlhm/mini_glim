#!/usr/bin/env python
import sys, subprocess
from pathlib import Path

def run(cmd): subprocess.run(cmd, check=True)

def find_svo(d: Path):
    for pat in ("*.svo2", "*.svo"):
        m = sorted(d.glob(pat))
        if m: return m[0]
    return None

def main():
    if len(sys.argv) < 2:
        print("usage: orchestrate.py <RUN_DIR> [SVO_PATH]")
        sys.exit(1)

    run_dir = Path(sys.argv[1]).expanduser().resolve()
    svo = Path(sys.argv[2]).expanduser().resolve() if len(sys.argv) > 2 else find_svo(run_dir)
    if not run_dir.is_dir(): sys.exit("RUN_DIR not found")
    if not svo or not svo.exists(): sys.exit("SVO not found")

    repo = Path(__file__).resolve().parent
    py = repo / "extraction" /".venv" / "bin" / "python"

    # 1) extract (in-place into RUN_DIR)
    run([str(py), str(repo / "extraction" / "run_extraction.py"), str(svo), str(run_dir)])

    # 2) GLIM + matching
    run(["bash", str(repo / "localization" / "go.sh"), str(run_dir)])

if __name__ == "__main__":
    main()
