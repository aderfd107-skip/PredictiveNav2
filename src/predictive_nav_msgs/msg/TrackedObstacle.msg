# One persistent obstacle Track at the shared TrackedObstacleArray timestamp.
# Position/velocity are expressed in the array header's frame (currently odom).
# confidence is a tracker freshness heuristic, not a calibrated probability.

uint32 track_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/Vector3 size
uint32 age
uint32 missed_frames
float32 confidence
