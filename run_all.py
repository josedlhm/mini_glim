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
        print("usage: orchestrate.py <RUN_DIR> [SVO_PATH] [MODEL_NAME]")
        sys.exit(1)

    run_dir = Path(sys.argv[1]).expanduser().resolve()
    svo = Path(sys.argv[2]).expanduser().resolve() if len(sys.argv) > 2 and not sys.argv[2].endswith(('.pth','.yaml')) else find_svo(run_dir)
    model_name = sys.argv[3] if len(sys.argv) > 3 else (None if len(sys.argv) <= 2 or not sys.argv[2] else (None if sys.argv[2].endswith(('.svo','.svo2')) else sys.argv[2]))

    if not run_dir.is_dir(): sys.exit("RUN_DIR not found")
    if not svo or not svo.exists(): sys.exit("SVO not found")

    repo = Path(__file__).resolve().parent

    # 1) extract (in-place)
    py_extract = repo / "extraction" / ".venv" / "bin" / "python"
    run([str(py_extract), str(repo / "extraction" / "run_extraction.py"), str(svo), str(run_dir)])

    # 2) GLIM + matching
    run(["bash", str(repo / "localization" / "go.sh"), str(run_dir)])

    # 3) detection (optional)
    # If detection venv/script exist, run it. Pass model_name if provided.
    detect_script = repo / "detection" / "make_sahi_detections_json.py"
    py_detect = repo / "detection" / ".venv" / "bin" / "python"
    if detect_script.exists() and py_detect.exists():
        cmd = [str(py_detect), str(detect_script), str(run_dir)]
        if model_name: cmd.append(model_name)
        run(cmd)

if __name__ == "__main__":
    main()
