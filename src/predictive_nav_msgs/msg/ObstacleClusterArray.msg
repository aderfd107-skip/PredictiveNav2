# All cluster observations from one LaserScan frame.
# header.frame_id is the coordinate frame used by every centroid, currently odom.
# header.stamp is copied from the source LaserScan measurement time.

std_msgs/Header header
ObstacleCluster[] clusters
