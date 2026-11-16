std_msgs/Header header
uint8 stage_id
# True only for a frozen selection captured after confirmed HOLDING.
# An empty valid array means four fruits were evaluated and none selected.
bool valid
HarvestTarget[] targets
# Copied from VisionStopState, so repeated trees in one route stage are distinct.
uint64 stop_id
