#!/usr/bin/env python3
"""
Scale an AprilTag image for printing without interpolation (nearest-neighbor).

Examples:
  python3 scale_apriltag.py tag.png --scale 10
  python3 scale_apriltag.py tag.png --width 1000 --output tag_print.png
"""

import argparse
import sys
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    print("ERROR: Pillow is required. Install with: pip install pillow", file=sys.stderr)
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Scale an AprilTag image using nearest-neighbor (no interpolation)."
    )
    parser.add_argument("input", help="Input image path (PNG recommended)")
    parser.add_argument(
        "--output",
        "-o",
        help="Output image path (default: <input>_scaled.png)",
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--scale",
        type=int,
        help="Integer scale factor (e.g., 10 doubles each pixel 10x)",
    )
    group.add_argument(
        "--width",
        type=int,
        help="Target output width in pixels (height scaled to preserve aspect ratio)",
    )
    parser.add_argument(
        "--height",
        type=int,
        help="Optional target output height in pixels (must match aspect ratio if provided)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    in_path = Path(args.input)
    if not in_path.exists():
        print(f"ERROR: Input file not found: {in_path}", file=sys.stderr)
        sys.exit(1)

    out_path = Path(args.output) if args.output else in_path.with_name(f"{in_path.stem}_scaled.png")

    img = Image.open(in_path)
    src_w, src_h = img.size

    if args.scale is not None:
        if args.scale <= 0:
            print("ERROR: --scale must be > 0", file=sys.stderr)
            sys.exit(1)
        dst_w = src_w * args.scale
        dst_h = src_h * args.scale
    else:
        if args.width is None or args.width <= 0:
            print("ERROR: --width must be > 0", file=sys.stderr)
            sys.exit(1)
        dst_w = args.width
        # Preserve aspect ratio
        dst_h = round(src_h * (dst_w / src_w))
        if args.height is not None:
            if args.height <= 0:
                print("ERROR: --height must be > 0", file=sys.stderr)
                sys.exit(1)
            # Enforce exact aspect ratio if user supplies both.
            if args.height != dst_h:
                print(
                    f"ERROR: Requested --height {args.height} does not match aspect ratio "
                    f"(expected {dst_h} for width {dst_w}).",
                    file=sys.stderr,
                )
                sys.exit(1)
            dst_h = args.height

    # Nearest-neighbor preserves the hard edges required for AprilTags.
    scaled = img.resize((dst_w, dst_h), resample=Image.NEAREST)
    scaled.save(out_path)

    print(f"Input:  {in_path} ({src_w}x{src_h})")
    print(f"Output: {out_path} ({dst_w}x{dst_h})")
    print("Resample: NEAREST (no interpolation)")


if __name__ == "__main__":
    main()
