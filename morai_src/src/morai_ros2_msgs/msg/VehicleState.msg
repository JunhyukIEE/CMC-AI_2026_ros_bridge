std_msgs/Header header

# Vehicle position and orientation
float64 x
float64 y
float64 yaw

# Vehicle velocity
float64 velocity
float64 target_velocity

# Path tracking errors
float64 cross_track_error      # CTE [m]
float64 heading_error          # HE [rad]

# Waypoint information
int32 current_waypoint_index
int32 target_waypoint_index
float64 lookahead_distance

# Target waypoint position (for controller use)
float64 target_x
float64 target_y
float64 target_yaw
