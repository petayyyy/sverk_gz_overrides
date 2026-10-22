# Integrated observation from the image-based H-Flow approximation.
std_msgs/Header header
float64 integration_time_s
# Image feature motion: positive right / down, in radians over the interval.
float64[2] image_flow_rad
# Equivalent sensor RH angular motion in the local +X-forward FLU frame:
# (0, -image_flow_rad[1], image_flow_rad[0]). Includes rotational motion.
# This is NOT a translation/velocity estimate or a gyro-compensated signal.
geometry_msgs/Vector3 angular_displacement
uint8 quality
uint16 tracked_features
bool distance_valid
float64 distance_m
float64 max_flow_rate_rad_s
