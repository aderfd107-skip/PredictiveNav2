# One obstacle's short-horizon predicted trajectory.
# time_offsets[i] and poses[i] describe the same future instant.
# Every offset is relative to PredictedTrajectoryArray.header.stamp, and every
# pose is expressed in PredictedTrajectoryArray.header.frame_id (currently odom).

# Persistent ID inherited from TrackedObstacle.track_id.
uint32 track_id

# Ordered future time offsets, for example 0.2 s, 0.4 s, 0.6 s.
builtin_interfaces/Duration[] time_offsets

# Predicted pose and its 6x6 covariance at the corresponding time offset.
geometry_msgs/PoseWithCovariance[] poses

# Obstacle footprint copied from the associated tracked obstacle, in metres.
geometry_msgs/Vector3 size

# Freshness heuristic copied from the track. It is not a calibrated probability.
float32 confidence
