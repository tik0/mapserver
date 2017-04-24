# Inverse sensor model

Header                       header

# Probabilities in the grid in signed char notation [0 .. 100].
# Probabilitiy-values smaller than 0 are interpreted as unknown
int32[] mapInt


# Probabilities in the grid in floating single precission notation [0.0 .. 1.0].
# Probabilitiy-values smaller than 0 are interpreted as unknown
float32[] mapFloat

# Number of rows
int32 rows

# Number of cols
int32 cols

# Meter per grid tile
float32 resolution
