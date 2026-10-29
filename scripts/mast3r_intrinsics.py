#!/usr/bin/env python3
"""Estimate pinhole intrinsics for a SubT camera folder with MASt3R.

This is the optional "MASt3R initialisation" backend for ``2_subt_dataset``:
instead of being handed ``--fx/--fy/--cx/--cy``, the replay tool samples a few
frames, runs MASt3R on image pairs and recovers a focal length from the
predicted point maps. Main point is assumed to be the image centre, which is
what MASt3R itself assumes when it optimises a focal length.

Only torch/numpy/PIL are used, so no cv2/roma/sklearn is needed for this path.

Output (``key = value`` lines, ``#`` starts a comment) is written to ``--output``
or stdout, and parsed by ``examples/2_subt_dataset.cpp``:

    model = pinhole
    source = mast3r
    focal = 408.02
    width = 640
    height = 480

``focal`` is the **virtual pinhole focal length** the backend consumes, not the
MEI projection parameter: MASt3R fits a pinhole model to the predicted point
maps, so what it recovers is the effective pinhole ``f``. The replay tool turns
it into ``pinholeScale = focal / fx`` against the calibrated ``fx`` and inherits
``xi`` / distortion from the calibration, so the fisheye geometry is preserved.
Writing ``fx`` here instead would mislabel a pinhole focal as a MEI parameter and
degrade the distortion inversion.
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate pinhole intrinsics with MASt3R")
    parser.add_argument("--images", required=True, help="directory holding the frame images (cam_0)")
    parser.add_argument("--pairs", type=int, default=6, help="number of image pairs to average (default 6)")
    parser.add_argument("--gap", type=int, default=8, help="frame gap inside a pair (default 8)")
    parser.add_argument("--max-frames", type=int, default=0, help="only consider the first N frames (0 = all)")
    parser.add_argument("--weights", default="", help="MASt3R checkpoint; MAST3R_WEIGHTS is also honoured")
    parser.add_argument("--device", default="auto", help="auto | cpu | cuda (default auto)")
    parser.add_argument("--output", default="", help="write the result here instead of stdout")
    parser.add_argument("--verbose", action="store_true", help="print per-pair focal estimates")
    return parser.parse_args(argv)


def list_frames(image_dir: Path, max_frames: int) -> list[Path]:
    frames = sorted(p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    if not frames:
        raise SystemExit(f"no images found in {image_dir}")
    if max_frames > 0:
        frames = frames[:max_frames]
    return frames


def pick_pairs(frames: list[Path], count: int, gap: int) -> list[tuple[Path, Path]]:
    """Spread `count` pairs over the sequence so focal estimates see diverse views."""
    if len(frames) < 2:
        raise SystemExit("need at least two frames to estimate intrinsics")
    gap = max(1, min(gap, len(frames) - 1))
    usable = len(frames) - gap
    if usable <= 0:
        return [(frames[0], frames[-1])]
    step = max(1, usable // max(1, count))
    pairs = []
    for i in range(0, usable, step):
        pairs.append((frames[i], frames[i + gap]))
        if len(pairs) >= count:
            break
    return pairs


def load_models():
    """Import MASt3R lazily so --help stays usable without the heavy stack."""
    here = Path(__file__).resolve().parent
    repo = here.parent / "thirdparty" / "mast3r"
    dust3r_repo = repo / "dust3r"
    if not (dust3r_repo / "dust3r").is_dir():
        raise SystemExit(
            f"dust3r submodule missing under {dust3r_repo}\n"
            "run: git submodule update --init --recursive thirdparty/mast3r"
        )
    for path in (str(repo), str(dust3r_repo)):
        if path not in sys.path:
            sys.path.insert(0, path)

    import torch  # noqa: PLC0415
    from dust3r.post_process import estimate_focal_knowing_depth  # noqa: PLC0415
    from mast3r.model import load_model  # noqa: PLC0415

    return torch, estimate_focal_knowing_depth, load_model


def default_weights() -> Path:
    import os  # noqa: PLC0415

    env = os.environ.get("MAST3R_WEIGHTS", "")
    if env:
        return Path(env).expanduser()
    candidates = [
        Path(__file__).resolve().parent.parent
        / "thirdparty"
        / "mast3r"
        / "checkpoints"
        / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth",
        Path.home()
        / "schoolwork"
        / "mast3r"
        / "checkpoints"
        / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise SystemExit(
        "MASt3R checkpoint not found. Pass --weights PATH or set MAST3R_WEIGHTS.\n"
        "download: https://download.europe.naverlabs.com/ComputerVision/MASt3R/"
        "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth"
    )


def prepare_frame(path: Path, size: int, patch_size: int):
    """Mirror dust3r.utils.image.load_images for a single frame (no cv2 needed)."""
    import numpy as np  # noqa: PLC0415
    import PIL.Image  # noqa: PLC0415
    from PIL.ImageOps import exif_transpose  # noqa: PLC0415
    import torch  # noqa: PLC0415
    import torchvision.transforms as tvf  # noqa: PLC0415

    img = exif_transpose(PIL.Image.open(path)).convert("RGB")
    orig_w, orig_h = img.size
    long_edge = max(orig_w, orig_h)
    interp = PIL.Image.LANCZOS if long_edge > size else PIL.Image.BICUBIC
    new_size = tuple(int(round(x * size / long_edge)) for x in img.size)
    img = img.resize(new_size, interp)

    w, h = img.size
    cx, cy = w // 2, h // 2
    half_w = ((2 * cx) // patch_size) * patch_size / 2
    half_h = ((2 * cy) // patch_size) * patch_size / 2
    if w == h:  # dust3r crops non-square unless square_ok
        half_h = 3 * half_w / 4
    img = img.crop((int(cx - half_w), int(cy - half_h), int(cx + half_w), int(cy + half_h)))

    crop_w, crop_h = img.size
    norm = tvf.Compose([tvf.ToTensor(), tvf.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])
    view = {
        "img": norm(img)[None],
        # the model expects a tensor here (load_images hands over numpy and the
        # older code path converted it); keep it explicit
        "true_shape": torch.tensor([img.size[::-1]], dtype=torch.int32),
        "instance": str(path),
    }
    return view, (orig_w, orig_h), (crop_w, crop_h)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    image_dir = Path(args.images).expanduser()
    if not image_dir.is_dir():
        raise SystemExit(f"--images is not a directory: {image_dir}")

    torch, estimate_focal, load_model = load_models()

    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
    weights = Path(args.weights).expanduser() if args.weights else default_weights()
    if not weights.is_file():
        raise SystemExit(f"checkpoint not found: {weights}")

    frames = list_frames(image_dir, args.max_frames)
    pairs = pick_pairs(frames, max(1, args.pairs), args.gap)
    print(f"[mast3r] {len(frames)} frames, {len(pairs)} pairs, device={device}", file=sys.stderr, flush=True)
    print(f"[mast3r] loading {weights}", file=sys.stderr, flush=True)

    model = load_model(str(weights), device, verbose=False)
    model.eval()

    estimates: list[float] = []
    scale_ratios: list[float] = []
    orig_sizes: set[tuple[int, int]] = set()
    for index, (left, right) in enumerate(pairs):
        view1, orig1, crop1 = prepare_frame(left, 512, 16)
        view2, orig2, crop2 = prepare_frame(right, 512, 16)
        orig_sizes.add(orig1)
        orig_sizes.add(orig2)
        view1 = {k: (v.to(device) if hasattr(v, "to") else v) for k, v in view1.items()}
        view2 = {k: (v.to(device) if hasattr(v, "to") else v) for k, v in view2.items()}
        with torch.no_grad():
            res1, _ = model(view1, view2)
        crop_w, crop_h = crop1
        pp = torch.tensor((crop_w / 2, crop_h / 2), device=device)
        focal = float(
            estimate_focal(res1["pts3d"], pp, focal_mode="weiszfeld", min_focal=0.0, max_focal=1e5)
        )
        if not (focal > 0) or focal != focal:  # NaN or non-positive
            print(f"[mast3r] pair {index}: rejected (focal={focal})", file=sys.stderr, flush=True)
            continue
        # focal is in cropped/resized pixels; scale back to the raw frame
        ratio_x = orig1[0] / crop_w
        ratio_y = orig1[1] / crop_h
        scale_ratios.append(0.5 * (ratio_x + ratio_y))
        focal_orig = focal * scale_ratios[-1]
        estimates.append(focal_orig)
        if args.verbose:
            print(
                f"[mast3r] pair {index}: {left.name} vs {right.name} -> "
                f"f={focal:.2f}px@crop, {focal_orig:.2f}px@raw (scale {scale_ratios[-1]:.3f})",
                file=sys.stderr,
                flush=True,
            )

    if not estimates:
        raise SystemExit("MASt3R produced no usable focal estimate for this sequence")

    if len(orig_sizes) != 1:
        print(f"[mast3r] warning: frames have mixed sizes {sorted(orig_sizes)}", file=sys.stderr)
    width, height = sorted(orig_sizes)[0]

    # A single shared focal (square pixels) is what MASt3R's own optimiser uses.
    focal = statistics.median(estimates)
    lines = [
        "# MASt3R pinhole focal (image centre as principal point)",
        "# This is the *virtual pinhole focal length*, consumed as focal = fx * pinholeScale.",
        "# Distortion (xi / k1 / k2 / p1 / p2) is inherited from the calibration, not written here.",
        f"# pairs={len(estimates)} device={device} checkpoint={weights.name}",
        "model = pinhole",
        "source = mast3r",
        f"focal = {focal:.6f}",
        f"cx = {width / 2.0:.6f}",
        f"cy = {height / 2.0:.6f}",
        f"width = {width}",
        f"height = {height}",
    ]
    text = "\n".join(lines) + "\n"
    if args.output:
        out_path = Path(args.output).expanduser()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text, encoding="utf-8")
        print(f"[mast3r] wrote {out_path}", file=sys.stderr, flush=True)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
