# One terminal result for an entire frozen snapshot, after every round.
std_msgs/Header header
uint8 stage_id
uint64 stop_id
builtin_interfaces/Time snapshot_stamp
bool success
# True only after real successful home actions, never target-only or dry-run.
bool returned_home
bool dry_run
string message
