std_msgs/Header header
# Only feedback matching the current nonzero stage may end a route action.
uint8 stage_id
# Latched arrival, not the instantaneous GPIO input level.
bool reached
