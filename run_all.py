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
    model_name = sys.argv[3] if len(sys.argv) > 3 else None

    if not run_dir.is_dir(): sys.exit("RUN_DIR not found")
    if not svo or not svo.exists(): sys.exit("SVO not found")

    repo = Path(__file__).resolve().parent

    # 1) Extraction
    extract_success = run_dir / "images"
    if not extract_success.is_dir() or not any(extract_success.iterdir()):
        py_extract = repo / "extraction" / ".venv" / "bin" / "python"
        run([str(py_extract), str(repo / "extraction" / "run_extraction.py"), str(svo), str(run_dir)])
    else:
        print("[skip] extraction step already done")

    # 2) GLIM + matching
    glim_output = run_dir / "output" / "poses_at_frames.txt"
    if not glim_output.exists():
        run(["bash", str(repo / "localization" / "go.sh"), str(run_dir)])
    else:
        print("[skip] localization step already done")

    # 3) Detection
    det_success = run_dir / "detections" / "SUCCESS"
    detect_script = repo / "detection" / "make_sahi_detections_json.py"
    py_detect = repo / "detection" / ".venv" / "bin" / "python"
    if not det_success.exists() and detect_script.exists():
        cmd = [str(py_detect), str(detect_script), str(run_dir)]
        if model_name: cmd.append(model_name)
        run(cmd)
    else:
        print("[skip] detection step already done")

if __name__ == "__main__":
    main()
