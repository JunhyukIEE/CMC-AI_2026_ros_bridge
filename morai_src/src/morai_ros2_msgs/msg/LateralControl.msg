std_msgs/Header header

# Lateral control command
float64 steering_angle         # Steering angle [-1.0, 1.0]
float64 steering_angle_velocity  # Optional: rate of change

# Debug information
float64 curvature             # Path curvature at target point
float64 lateral_error         # Same as CTE for reference
