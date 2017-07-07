# A request structure for a ism or rsm as primitive

Header                       header

# Number of tiles in x direction
# Values smaller then 2 are interpreted as omitted dimension
int32 width

# Number of tiles in y direction
# Values smaller then 2 are interpreted as omitted dimension
int32 depth

# Number of tiles in z direction
# Values smaller then 2 are interpreted as omitted dimension
int32 height

# Parent frame of the request
string frame_id

# Pose of the request in frame_id
geometry_msgs/PoseStamped pose

# Requested meter per grid tile
# Values smaller than 0 are interpreted as original resolution
float32 resolution

# Additional request information
string req_info

# Action to perform on the data
string action