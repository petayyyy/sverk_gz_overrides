import math

import cv2
import numpy as np
import pytest

from mtf01_px4_bridge.flow import estimate_flow


def textured_image() -> np.ndarray:
    image = np.zeros((100, 100), dtype=np.uint8)
    for y in range(10, 91, 10):
        for x in range(10, 91, 10):
            cv2.circle(image, (x, y), 2, 255, -1)
    return image


def shifted(image: np.ndarray, x: float, y: float) -> np.ndarray:
    transform = np.float32([[1, 0, x], [0, 1, y]])
    return cv2.warpAffine(image, transform, (image.shape[1], image.shape[0]))


def test_estimates_px4_opencv_flow_convention():
    previous = textured_image()
    estimate = estimate_flow(previous, shifted(previous, 2.0, -3.0), math.radians(42))
    focal = 50.0 / math.tan(math.radians(21))

    assert estimate.quality > 0
    assert estimate.tracked_features >= 6
    assert estimate.integrated_x == pytest.approx(math.atan2(2.0, focal), abs=0.005)
    assert estimate.integrated_y == pytest.approx(math.atan2(-3.0, focal), abs=0.005)


def test_textureless_frame_has_zero_quality():
    image = np.zeros((100, 100), dtype=np.uint8)
    estimate = estimate_flow(image, image, math.radians(42))

    assert estimate.quality == 0
    assert estimate.integrated_x == 0.0
    assert estimate.integrated_y == 0.0
