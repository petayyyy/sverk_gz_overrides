from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class FlowEstimate:
    integrated_x: float
    integrated_y: float
    quality: int
    tracked_features: int


def estimate_flow(
    previous: np.ndarray,
    current: np.ndarray,
    horizontal_fov: float,
    *,
    max_features: int = 80,
) -> FlowEstimate:
    """Estimate integrated angular image flow using PX4's OpenCV convention."""
    if previous.ndim != 2 or current.ndim != 2:
        raise ValueError("flow images must be single-channel")
    if previous.shape != current.shape:
        raise ValueError("flow images must have identical dimensions")
    if not 0.0 < horizontal_fov < math.pi:
        raise ValueError("horizontal_fov must be between 0 and pi")
    if max_features < 8:
        raise ValueError("max_features must be at least 8")

    features = cv2.goodFeaturesToTrack(
        previous,
        maxCorners=max_features,
        qualityLevel=0.01,
        minDistance=4,
        blockSize=5,
    )
    if features is None or len(features) < 6:
        return FlowEstimate(0.0, 0.0, 0, 0)

    tracked, status, errors = cv2.calcOpticalFlowPyrLK(
        previous,
        current,
        features,
        None,
        winSize=(15, 15),
        maxLevel=2,
        criteria=(
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            20,
            0.03,
        ),
    )
    if tracked is None or status is None:
        return FlowEstimate(0.0, 0.0, 0, 0)

    valid = status.reshape(-1).astype(bool)
    if errors is not None:
        valid &= np.isfinite(errors.reshape(-1))
    displacements = tracked.reshape(-1, 2)[valid] - features.reshape(-1, 2)[valid]
    if len(displacements) < 6:
        return FlowEstimate(0.0, 0.0, 0, int(len(displacements)))

    median = np.median(displacements, axis=0)
    residual = np.linalg.norm(displacements - median, axis=1)
    median_residual = float(np.median(residual))
    threshold = max(0.75, 3.0 * median_residual)
    inliers = displacements[residual <= threshold]
    if len(inliers) < 6:
        return FlowEstimate(0.0, 0.0, 0, int(len(inliers)))

    pixel_flow = np.mean(inliers, axis=0)
    focal_length = (previous.shape[1] / 2.0) / math.tan(horizontal_fov / 2.0)
    integrated_x = math.atan2(float(pixel_flow[0]), focal_length)
    integrated_y = math.atan2(float(pixel_flow[1]), focal_length)
    quality = int(round(255.0 * min(1.0, len(inliers) / float(max_features))))
    return FlowEstimate(
        integrated_x,
        integrated_y,
        max(1, min(255, quality)),
        int(len(inliers)),
    )
