#!/usr/bin/env python3
import sys, json
from pathlib import Path
import cv2
import numpy as np
from sahi import AutoDetectionModel
from sahi.predict import get_sliced_prediction

CONF_THRESH = 0.5
DEVICE = "cpu"  # "cuda" if available

# slicing parameters
SLICE_HEIGHT = 720
SLICE_WIDTH  = 1280
OVERLAP_H    = 0.2
OVERLAP_W    = 0.2
POSTPROC     = "GREEDYNMM"

DETS_JSON = "detections.json"
WRITE_ANNOTATED = True
ANNOTATED_SUBDIR = "annotated"

FIXED_CLASS_COLORS = {
    1: (0, 0, 255),
    2: (255, 255, 255),
    3: (255, 0, 0),
    4: (0, 165, 255),
    5: (255, 0, 255),
    6: (0, 255, 0),
}

def color_for_class_id(cid: int):
    if cid in FIXED_CLASS_COLORS:
        return FIXED_CLASS_COLORS[cid]
    rng = np.random.default_rng(int(cid) & 0xFFFFFFFF)
    return tuple(int(x) for x in rng.integers(50, 255, size=3))

EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

# -----------------------------------------------------------------------------

def detect_on_run_dir(model: AutoDetectionModel, run_dir: Path):
    img_dir = run_dir / "images"
    out_dir = run_dir / "detections"
    success_f = out_dir / "SUCCESS"

    if success_f.exists():
        print(f"[detect] SKIP (already done): {run_dir.name}")
        return

    if not img_dir.is_dir():
        print(f"[detect] No images/ found in {run_dir}")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    if WRITE_ANNOTATED:
        (out_dir / ANNOTATED_SUBDIR).mkdir(exist_ok=True)

    all_dets = {}
    for img_path in sorted(p for p in img_dir.iterdir() if p.suffix.lower() in EXTS):
        img_bgr = cv2.imread(str(img_path))
        if img_bgr is None:
            continue
        H, W = img_bgr.shape[:2]
        vis = img_bgr.copy()

        res = get_sliced_prediction(
            image=img_bgr[..., ::-1],
            detection_model=model,
            slice_height=SLICE_HEIGHT,
            slice_width=SLICE_WIDTH,
            overlap_height_ratio=OVERLAP_H,
            overlap_width_ratio=OVERLAP_W,
            postprocess_type=POSTPROC,
            postprocess_class_agnostic=True,
        )

        dets_for_image = []
        for p in res.object_prediction_list:
            x1, y1 = int(p.bbox.minx), int(p.bbox.miny)
            x2, y2 = int(p.bbox.maxx), int(p.bbox.maxy)
            x1, y1, x2, y2 = max(0,x1), max(0,y1), min(W-1,x2), min(H-1,y2)
            mask = (p.mask.bool_mask.astype(np.uint8)*255) if getattr(p,"mask",None) else np.zeros((H,W),np.uint8)
            cropped = mask[y1:y2+1, x1:x2+1].astype(np.uint8)
            cls_id = int(getattr(p.category,"id",0))
            score = float(getattr(p.score,"value",p.score))
            dets_for_image.append({
                "bbox":[x1,y1,x2,y2],
                "score":score,
                "class_id":cls_id,
                "mask":cropped.tolist(),
            })
            if WRITE_ANNOTATED and mask.any():
                contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(vis, contours, -1, color_for_class_id(cls_id), 1)

        if WRITE_ANNOTATED:
            cv2.imwrite(str(out_dir / ANNOTATED_SUBDIR / img_path.name), vis)

        all_dets[img_path.name] = dets_for_image

    (out_dir / DETS_JSON).write_text(json.dumps(all_dets, indent=2))
    success_f.write_text("")
    print(f"[detect] Wrote: {out_dir / DETS_JSON}")

# -----------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python make_sahi_detections_json.py <RUN_DIR> [MODEL_NAME]", file=sys.stderr)
        sys.exit(1)

    run_dir = Path(sys.argv[1]).expanduser().resolve()
    model_name = sys.argv[2] if len(sys.argv) > 2 else "arandanos_casa_2"
    model_dir = Path(__file__).resolve().parent / "model_weights"

    weights = model_dir / f"{model_name}.pth"
    config  = model_dir / f"{model_name}.yaml"

    if not weights.exists() or not config.exists():
        sys.exit(f"[detect] model files not found in {model_dir}: {model_name}.pth/.yaml")

    print(f"[detect] Using model: {model_name}")
    model = AutoDetectionModel.from_pretrained(
        model_type="detectron2",
        model_path=str(weights),
        config_path=str(config),
        confidence_threshold=CONF_THRESH,
        device=DEVICE,
    )

    detect_on_run_dir(model, run_dir)

if __name__ == "__main__":
    main()
