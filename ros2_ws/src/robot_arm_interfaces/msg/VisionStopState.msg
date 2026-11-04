uint8 IDLE=0
uint8 SEARCHING=1
uint8 ALIGNING=2
uint8 SETTLING=3
uint8 HOLDING=4
uint8 FAULT=5
std_msgs/Header header
uint8 stage_id
uint8 state
# Legacy metric error, unused and left zero by image-space centering.
float64 tree_error_m
string message
# Image-space centering error, positive image-right; not meters.
float64 tree_center_error_ratio
