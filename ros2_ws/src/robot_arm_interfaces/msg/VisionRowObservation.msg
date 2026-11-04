# Acquisition time of the camera frame, before inference.
std_msgs/Header header
uint32 frame_sequence
bool mango_in_range
uint32 mango_count_in_range
float64 nearest_mango_depth_m
bool tree_valid
# Legacy metric error, unused and left zero by image-space centering.
float64 tree_lateral_error_m
# (trunk bounding-box center X - target image X) / displayed image width.
# Uses the rotated display image; positive is image-right, negative image-left.
# Only meaningful when tree_valid=true. Independent of the arm calibration.
float64 tree_center_error_ratio
float32 tree_confidence
