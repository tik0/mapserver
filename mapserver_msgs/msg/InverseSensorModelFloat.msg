# Inverse sensor model

std_msgs/Header header
nav_msgs/MapMetaData info

# Probabilities in the grid in floating single precission notation [0.0 .. 1.0].
# Probabilitiy-values smaller than 0 are interpreted as unknown
float32[] map
