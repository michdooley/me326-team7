# Depth-to-RGB Alignment

## Manual alignment using `image_raw`

If you need to align the raw depth topic `/camera/depth/image_raw` to the RGB frame yourself, use the utility script. D435 intrinsics are baked in as defaults:

```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'utilities'))
from align_depth_to_rgb import align_depth

aligned_depth = align_depth(depth, rgb)
```

## Via ROS2 (real hardware)

The launch files (`real.launch.py`, `robot.launch.py`) now enable the RealSense SDK's built-in depth alignment. An already-aligned depth stream is published to:

```
/camera/depth/aligned_image_raw
```

Can just subscribe to that topic instead of `/camera/depth/image_raw`.
