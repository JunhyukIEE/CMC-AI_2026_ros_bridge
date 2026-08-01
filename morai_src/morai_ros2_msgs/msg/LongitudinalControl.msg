std_msgs/Header header

# Longitudinal control commands
float64 accel                  # Acceleration command [0.0, 1.0]
float64 brake                  # Brake command [0.0, 1.0]

# Debug information
float64 velocity_error        # target_velocity - current_velocity
float64 acceleration_command  # Actual acceleration in m/s^2
