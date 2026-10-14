"""ROS-independent image flow, timing, range and synthetic noise utilities."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class FlowEstimate:
    right: float = 0.0
    down: float = 0.0
    quality: int = 0
    tracked: int = 0

def estimate_flow(previous, current, horizontal_fov_rad, max_features=160):
    import cv2
    if previous.dtype != np.uint8 or current.dtype != np.uint8 or previous.ndim != 2 or current.ndim != 2:
        raise ValueError('Flow images must be 2D uint8 grayscale arrays')
    if previous.shape != current.shape or min(previous.shape) < 16:
        raise ValueError('Flow images must have equal dimensions, at least 16x16')
    if not 0 < horizontal_fov_rad < math.pi or max_features < 8:
        raise ValueError('Invalid FoV or feature limit')
    features = cv2.goodFeaturesToTrack(previous, max_features, 0.02, 4, blockSize=5)
    if features is None or len(features) < 6:
        return FlowEstimate()
    tracked, status, errors = cv2.calcOpticalFlowPyrLK(previous, current, features, None,
                                                    winSize=(15, 15), maxLevel=2)
    if tracked is None:
        return FlowEstimate()
    backwards, backwards_status, _ = cv2.calcOpticalFlowPyrLK(current, previous, tracked, None,
                                                              winSize=(15, 15), maxLevel=2)
    if backwards is None:
        return FlowEstimate()
    a, b, back = features.reshape(-1, 2), tracked.reshape(-1, 2), backwards.reshape(-1, 2)
    h, w = previous.shape
    good = ((status.ravel() != 0) & (backwards_status.ravel() != 0)
            & np.isfinite(b).all(axis=1) & np.isfinite(back).all(axis=1)
            & (np.linalg.norm(back - a, axis=1) < 1.0)
            & (b[:, 0] >= 0) & (b[:, 0] < w) & (b[:, 1] >= 0) & (b[:, 1] < h))
    if errors is not None:
        good &= np.isfinite(errors.ravel()) & (errors.ravel() < 30)
    a, b = a[good], b[good]
    if len(a) < 6:
        return FlowEstimate(tracked=len(a))
    displacement = b - a
    centre = np.median(displacement, axis=0)
    residual = np.linalg.norm(displacement - centre, axis=1)
    inliers = residual <= max(0.75, 3 * float(np.median(residual)))
    a, b = a[inliers], b[inliers]
    if len(a) < 6:
        return FlowEstimate(tracked=len(a))
    focal = (w / 2) / math.tan(horizontal_fov_rad / 2)
    principal = np.array([(w - 1) / 2, (h - 1) / 2])
    angular = np.arctan((b - principal) / focal) - np.arctan((a - principal) / focal)
    right, down = np.median(angular, axis=0)
    quality = int(np.clip(round(255 * len(a) / max_features), 1, 255))
    return FlowEstimate(float(right), float(down), quality, len(a))

def image_flow_to_flu(right, down):
    """Positive output follows RH sensor rotation viewing a stationary scene."""
    return np.array([0.0, -down, right], dtype=float)

def scalar_range(values, lower, upper):
    if not 0 < lower < upper:
        raise ValueError('Range bounds must satisfy 0 < min < max')
    a = np.asarray(values, dtype=float)
    good = a[np.isfinite(a) & (a >= lower) & (a <= upper)]
    if len(good):
        return float(np.median(good))
    if np.any(np.isneginf(a)) or np.any(np.isfinite(a) & (a < lower)):
        return -math.inf
    if np.any(np.isposinf(a)) or np.any(np.isfinite(a) & (a > upper)):
        return math.inf
    return math.nan

def fresh_range(sample, image_stamp_ns, max_age_s, lower, upper):
    if sample is None:
        return math.nan
    stamp, value = sample
    age = (image_stamp_ns - stamp) / 1e9
    return value if 0 <= age <= max_age_s and math.isfinite(value) and lower <= value <= upper else math.nan

class FlowNoise:
    """Synthetic white rate noise plus a fixed-per-run rate bias, both in rad/s."""
    def __init__(self, white_rate, bias_rate, seed):
        if not math.isfinite(white_rate) or not math.isfinite(bias_rate) or white_rate < 0 or bias_rate < 0:
            raise ValueError('Noise standard deviations must be finite and nonnegative')
        self.rng = np.random.default_rng(seed)
        self.white_rate = white_rate
        self.bias = self.rng.normal(0, bias_rate, 2)

    def apply(self, integrated_flow, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Integration time must be finite and positive')
        return np.asarray(integrated_flow) + dt * (self.bias + self.rng.normal(0, self.white_rate, 2))
