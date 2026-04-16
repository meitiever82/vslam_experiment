"""Thread-safe frame buffer used by the droid_w_ros2 node.

The ROS subscription callback runs on the rclpy executor thread; the buffer
must be safe to push from that thread and snapshot from the SLAM-runner
thread. We keep frames as plain (timestamp_sec, np.ndarray BGR uint8) tuples
so the snapshot can be handed straight to BufferedRGB.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

Frame = Tuple[float, np.ndarray]


class FrameBuffer:
    def __init__(self, max_frames: int = 0):
        """max_frames=0 disables the cap (grow unbounded)."""
        self._lock = threading.Lock()
        self._frames: deque = deque(maxlen=max_frames if max_frames > 0 else None)
        self._frozen = False

    def push(self, timestamp: float, image_bgr: np.ndarray) -> bool:
        """Append a frame. Returns False if the buffer has been frozen."""
        with self._lock:
            if self._frozen:
                return False
            # Copy so the upstream cv_bridge buffer can be reused.
            self._frames.append((float(timestamp), image_bgr.copy()))
            return True

    def freeze(self) -> List[Frame]:
        """Stop accepting new frames and return a snapshot of what we have."""
        with self._lock:
            self._frozen = True
            return list(self._frames)

    def snapshot(self) -> List[Frame]:
        """Return a copy of current frames without freezing."""
        with self._lock:
            return list(self._frames)

    def clear(self) -> None:
        with self._lock:
            self._frames.clear()
            self._frozen = False

    def __len__(self) -> int:
        with self._lock:
            return len(self._frames)

    @property
    def frozen(self) -> bool:
        with self._lock:
            return self._frozen

    def latest_timestamp(self) -> Optional[float]:
        with self._lock:
            if not self._frames:
                return None
            return self._frames[-1][0]
