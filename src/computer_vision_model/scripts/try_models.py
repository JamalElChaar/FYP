#!/usr/bin/env python3
"""Run several Roboflow models against one saved image and compare them.

Swapping models through the full pipeline costs a launch, a camera start and
a 10 s settle each time. This hits the same inference API directly on an
image already captured, so several candidates can be compared in seconds.

    export ROBOFLOW_API_KEY=...
    python3 try_models.py <image.jpg> [model_id ...]

With no model ids it tries a default shortlist of whole-banana and general
fruit detectors. Annotated results are written next to the image as
<stem>.<model>.jpg so the boxes can be compared by eye.
"""

import os
import sys
from pathlib import Path

DEFAULT_MODELS = [
    "sliced-fruits-and-vegetables-rnw8f/1",   # what the pipeline uses today
    "banana-qn9ae/1",                          # whole banana, object detection
    "banana123/11",                            # banana-specific
    "fruits-detection-vlos6/1",                # 6 fruit classes incl. banana
]


def annotate(image_path, model_id, predictions, out_path):
    import cv2
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return None
    best = max(predictions, key=lambda p: p.get("confidence", 0.0), default=None)
    for p in predictions:
        cx, cy = float(p["x"]), float(p["y"])
        w, h = float(p["width"]), float(p["height"])
        x0, y0 = int(cx - w / 2), int(cy - h / 2)
        x1, y1 = int(cx + w / 2), int(cy + h / 2)
        chosen = p is best
        colour = (0, 215, 255) if chosen else (120, 120, 120)
        cv2.rectangle(image, (x0, y0), (x1, y1), colour, 2 if chosen else 1)
        caption = f"{p.get('class','?')} {p.get('confidence',0):.2f}"
        ty = y0 - 6 if y0 - 6 > 10 else min(y1 + 16, image.shape[0] - 4)
        cv2.putText(image, caption, (max(x0, 2), ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, caption, (max(x0, 2), ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
    header = f"{model_id}  -  {len(predictions)} detection(s)"
    cv2.putText(image, header, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, header, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out_path), image)
    return out_path


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    image_path = Path(sys.argv[1]).expanduser().resolve()
    if not image_path.is_file():
        print(f"error: no such image: {image_path}")
        return 1
    models = sys.argv[2:] or DEFAULT_MODELS

    api_key = os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        print("error: ROBOFLOW_API_KEY is not set")
        return 1

    from inference_sdk import InferenceHTTPClient
    client = InferenceHTTPClient(api_url="https://serverless.roboflow.com",
                                 api_key=api_key)

    print(f"image: {image_path}\n")
    for model_id in models:
        try:
            raw = client.infer(str(image_path), model_id=model_id)
        except Exception as error:
            print(f"  {model_id:44} FAILED: {type(error).__name__}: {str(error)[:60]}")
            continue
        preds = raw.get("predictions", []) if isinstance(raw, dict) else []
        if not preds:
            print(f"  {model_id:44} 0 detections")
            continue
        preds.sort(key=lambda p: p.get("confidence", 0.0), reverse=True)
        top = preds[0]
        print(f"  {model_id:44} {len(preds)} detection(s)   "
              f"best: {top.get('class','?')} {top.get('confidence',0):.3f} "
              f"at ({top['x']:.0f},{top['y']:.0f})")
        for p in preds[1:4]:
            print(f"  {'':44}   also: {p.get('class','?')} {p.get('confidence',0):.3f}")
        safe = model_id.replace("/", "_")
        out = image_path.with_suffix(f".{safe}.jpg")
        if annotate(image_path, model_id, preds, out):
            print(f"  {'':44}   annotated -> {out.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
