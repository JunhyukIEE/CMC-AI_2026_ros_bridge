#ifndef WAYPOINT_FOLLOWER_BASE__TRAJECTORY_MANAGER_HPP_
#define WAYPOINT_FOLLOWER_BASE__TRAJECTORY_MANAGER_HPP_

#include <vector>
#include <string>
#include <cmath>
#include <limits>

namespace waypoint_follower
{

struct Waypoint
{
  double x;
  double y;
  double yaw;
  double velocity;
};

struct VehiclePose
{
  double x;
  double y;
  double yaw;
  double velocity;
};

struct TrackingError
{
  double cross_track_error;    // CTE [m]
  double heading_error;         // HE [rad]
  int closest_waypoint_index;
  int target_waypoint_index;
};

class TrajectoryManager
{
public:
  TrajectoryManager();
  ~TrajectoryManager() = default;

  struct RandomizationParams
  {
    bool enabled = false;
    double lateral_offset_max = 0.5;  // [m] max left/right offset
    double speed_min = 0.0;           // [m/s]
    double speed_max = 11.0;          // [m/s]
    int smoothing_window = 1;         // odd >=1, 1 = no smoothing
    int seed = 0;                     // 0 = random_device
    bool recompute_yaw = true;
  };

  // Load waypoints from CSV file
  bool load_waypoints(const std::string & file_path);
  bool load_waypoints(const std::string & file_path, const RandomizationParams & params);

  // Get total number of waypoints
  size_t get_waypoint_count() const { return waypoints_.size(); }

  // Get specific waypoint
  const Waypoint & get_waypoint(size_t index) const { return waypoints_[index]; }

  // Get all waypoints
  const std::vector<Waypoint> & get_waypoints() const { return waypoints_; }

  // Find the closest waypoint to current position
  int find_closest_waypoint(double x, double y) const;

  // Find target waypoint based on lookahead distance
  int find_target_waypoint(
    const VehiclePose & current_pose,
    int start_index,
    double lookahead_distance) const;

  // Calculate CTE and HE
  TrackingError calculate_tracking_error(
    const VehiclePose & current_pose,
    double lookahead_distance) const;

  // Utility: normalize angle to [-pi, pi]
  static double normalize_angle(double angle);

  // Utility: calculate distance between two points
  static double calculate_distance(double x1, double y1, double x2, double y2);

private:
  std::vector<Waypoint> waypoints_;

  void apply_randomization(const RandomizationParams & params);
  static int clamp_window(int window);
  static double clamp_value(double value, double min_value, double max_value);

  // Calculate CTE as perpendicular distance from line segment
  double calculate_cte(
    const VehiclePose & pose,
    const Waypoint & wp1,
    const Waypoint & wp2) const;
};

}  // namespace waypoint_follower

#endif  // WAYPOINT_FOLLOWER_BASE__TRAJECTORY_MANAGER_HPP_
