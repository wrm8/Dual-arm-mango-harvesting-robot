# Stage and velocity travel together to avoid cross-topic switching races.
std_msgs/Header header
# 1..15: mission stage; 0: manual/unassigned, no photoelectric arrival.
uint8 stage_id
geometry_msgs/Twist velocity
