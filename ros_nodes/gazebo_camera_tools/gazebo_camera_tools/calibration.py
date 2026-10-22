"""Gazebo reports SDF/OpenGL principal points; ROS uses integer pixel centres."""
import copy

def to_opencv(message):
    result = copy.deepcopy(message)
    # An uncalibrated CameraInfo has K[0] == 0: preserve that sentinel.
    if result.k[0] != 0:
        result.k[2] -= 0.5
        result.k[5] -= 0.5
        result.p[2] -= 0.5
        result.p[6] -= 0.5
    return result
