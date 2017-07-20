# Point-NavSat-String tuple

Header                 header

# Point
geometry_msgs/Point    point
# Corresponding NavSat position
sensor_msgs/NavSatFix  navsat
# Corresponding global orientation for NavSatFix 
float64                yaw
# Corresponding tf name
std_msgs/String        string
