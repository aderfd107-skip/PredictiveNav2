# All valid obstacle trajectory predictions produced from one track update.
# header.stamp is the source track timestamp (the "now" of this prediction).
# header.frame_id is the common coordinate frame of every predicted pose,
# currently odom.

std_msgs/Header header
PredictedTrajectory[] trajectories
