#include "waypoint_follower_base/trajectory_manager.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <random>

namespace waypoint_follower
{

TrajectoryManager::TrajectoryManager()
{
}

bool TrajectoryManager::load_waypoints(const std::string & file_path)
{
  RandomizationParams params;
  params.enabled = false;
  return load_waypoints(file_path, params);
}

bool TrajectoryManager::load_waypoints(
  const std::string & file_path,
  const RandomizationParams & params)
{
  waypoints_.clear();

  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open waypoint file: " << file_path << std::endl;
    return false;
  }

  std::string line;
  // Skip header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    Waypoint wp;
    char comma;
    std::istringstream ss(line);
    ss >> wp.x >> comma >> wp.y >> comma >> wp.yaw >> comma >> wp.velocity;
    waypoints_.push_back(wp);
  }

  file.close();
  if (!waypoints_.empty() && params.enabled) {
    apply_randomization(params);
  }
  return !waypoints_.empty();
}

int TrajectoryManager::find_closest_waypoint(double x, double y) const
{
  if (waypoints_.empty()) {
    return -1;
  }

  double min_dist_sq = std::numeric_limits<double>::max();
  int closest_index = -1;

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    double dx = waypoints_[i].x - x;
    double dy = waypoints_[i].y - y;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_index = static_cast<int>(i);
    }
  }

  return closest_index;
}

int TrajectoryManager::find_target_waypoint(
  const VehiclePose & current_pose,
  int start_index,
  double lookahead_distance) const
{
  if (waypoints_.empty() || start_index < 0) {
    return -1;
  }

  const size_t waypoint_count = waypoints_.size();

  // Limit search to next 100 waypoints to prevent jumping to opposite side of track
  const size_t search_limit = std::min(static_cast<size_t>(100), waypoint_count);

  // Search from start_index forward (limited range)
  for (size_t i = 0; i < search_limit; ++i) {
    int check_index = (start_index + i) % waypoint_count;
    double dist = calculate_distance(
      waypoints_[check_index].x, waypoints_[check_index].y,
      current_pose.x, current_pose.y
    );

    if (dist >= lookahead_distance) {
      // Check if point is generally ahead of vehicle
      double path_yaw = std::atan2(
        waypoints_[check_index].y - current_pose.y,
        waypoints_[check_index].x - current_pose.x
      );
      double angle_diff = normalize_angle(path_yaw - current_pose.yaw);

      // Point should be within 90 degrees in front
      if (std::abs(angle_diff) < M_PI_2) {
        return check_index;
      }
    }
  }

  // Fallback: return a few waypoints ahead (not full loop)
  return (start_index + 5) % waypoint_count;
}

TrackingError TrajectoryManager::calculate_tracking_error(
  const VehiclePose & current_pose,
  double lookahead_distance) const
{
  TrackingError error;
  error.cross_track_error = 0.0;
  error.heading_error = 0.0;
  error.closest_waypoint_index = -1;
  error.target_waypoint_index = -1;

  if (waypoints_.empty()) {
    return error;
  }

  // Find closest waypoint
  error.closest_waypoint_index = find_closest_waypoint(current_pose.x, current_pose.y);

  if (error.closest_waypoint_index < 0) {
    return error;
  }

  // Find target waypoint based on lookahead
  error.target_waypoint_index = find_target_waypoint(
    current_pose,
    error.closest_waypoint_index,
    lookahead_distance
  );

  // Calculate CTE using line segment from closest to next waypoint
  int next_index = (error.closest_waypoint_index + 1) % waypoints_.size();
  const Waypoint & wp1 = waypoints_[error.closest_waypoint_index];
  const Waypoint & wp2 = waypoints_[next_index];

  error.cross_track_error = calculate_cte(current_pose, wp1, wp2);

  // Calculate heading error
  // Use the direction from closest waypoint to next waypoint
  double path_yaw = std::atan2(wp2.y - wp1.y, wp2.x - wp1.x);
  error.heading_error = normalize_angle(path_yaw - current_pose.yaw);

  return error;
}

double TrajectoryManager::calculate_cte(
  const VehiclePose & pose,
  const Waypoint & wp1,
  const Waypoint & wp2) const
{
  // Calculate CTE as perpendicular distance from line segment (wp1, wp2)

  // Vector from wp1 to wp2
  double dx = wp2.x - wp1.x;
  double dy = wp2.y - wp1.y;

  // Vector from wp1 to vehicle
  double vx = pose.x - wp1.x;
  double vy = pose.y - wp1.y;

  // Handle degenerate case where waypoints are the same
  double segment_length_sq = dx * dx + dy * dy;
  if (segment_length_sq < 1e-6) {
    return std::hypot(vx, vy);
  }

  // Project vehicle position onto line segment
  double t = (vx * dx + vy * dy) / segment_length_sq;
  t = std::max(0.0, std::min(1.0, t));  // Clamp to [0, 1]

  // Find closest point on segment
  double closest_x = wp1.x + t * dx;
  double closest_y = wp1.y + t * dy;

  // Calculate perpendicular distance
  double error_x = pose.x - closest_x;
  double error_y = pose.y - closest_y;
  double distance = std::hypot(error_x, error_y);

  // Determine sign of CTE (positive = left of path, negative = right)
  // Cross product: (wp2 - wp1) × (pose - wp1)
  double cross = dx * vy - dy * vx;

  return (cross >= 0) ? distance : -distance;
}

double TrajectoryManager::normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double TrajectoryManager::calculate_distance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

int TrajectoryManager::clamp_window(int window)
{
  if (window < 1) {
    return 1;
  }
  if (window % 2 == 0) {
    return window + 1;
  }
  return window;
}

double TrajectoryManager::clamp_value(double value, double min_value, double max_value)
{
  if (min_value > max_value) {
    std::swap(min_value, max_value);
  }
  return std::max(min_value, std::min(max_value, value));
}

void TrajectoryManager::apply_randomization(const RandomizationParams & params)
{
  if (waypoints_.empty()) {
    return;
  }

  const int window = clamp_window(params.smoothing_window);
  const int half_window = window / 2;
  const size_t count = waypoints_.size();

  std::mt19937 rng;
  if (params.seed == 0) {
    std::random_device rd;
    rng.seed(rd());
  } else {
    rng.seed(static_cast<uint32_t>(params.seed));
  }

  std::uniform_real_distribution<double> lateral_dist(
    -std::abs(params.lateral_offset_max),
    std::abs(params.lateral_offset_max));
  std::uniform_real_distribution<double> speed_dist(
    std::min(params.speed_min, params.speed_max),
    std::max(params.speed_min, params.speed_max));

  std::vector<double> offsets(count, 0.0);
  std::vector<double> speeds(count, 0.0);

  for (size_t i = 0; i < count; ++i) {
    offsets[i] = lateral_dist(rng);
    speeds[i] = speed_dist(rng);
  }

  if (window > 1) {
    std::vector<double> offsets_smoothed(count, 0.0);
    std::vector<double> speeds_smoothed(count, 0.0);

    for (size_t i = 0; i < count; ++i) {
      double offset_sum = 0.0;
      double speed_sum = 0.0;
      for (int k = -half_window; k <= half_window; ++k) {
        size_t idx = (static_cast<long>(i) + k + static_cast<long>(count)) % count;
        offset_sum += offsets[idx];
        speed_sum += speeds[idx];
      }
      offsets_smoothed[i] = offset_sum / static_cast<double>(window);
      speeds_smoothed[i] = speed_sum / static_cast<double>(window);
    }

    offsets.swap(offsets_smoothed);
    speeds.swap(speeds_smoothed);
  }

  for (size_t i = 0; i < count; ++i) {
    auto & wp = waypoints_[i];
    const double offset = offsets[i];
    const double sin_yaw = std::sin(wp.yaw);
    const double cos_yaw = std::cos(wp.yaw);

    // Left normal (x' = x - sin(yaw)*d, y' = y + cos(yaw)*d)
    wp.x = wp.x - sin_yaw * offset;
    wp.y = wp.y + cos_yaw * offset;
    wp.velocity = clamp_value(speeds[i], params.speed_min, params.speed_max);
  }

  if (params.recompute_yaw && count >= 2) {
    for (size_t i = 0; i < count; ++i) {
      const size_t next = (i + 1) % count;
      const double dx = waypoints_[next].x - waypoints_[i].x;
      const double dy = waypoints_[next].y - waypoints_[i].y;
      waypoints_[i].yaw = std::atan2(dy, dx);
    }
  }
}

}  // namespace waypoint_follower
