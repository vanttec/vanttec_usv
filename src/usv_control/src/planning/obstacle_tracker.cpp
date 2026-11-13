#include "usv_control/planning/obstacle_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Rotación del marco del bote al NED por el yaw ψ, más la traslación a la posición del bote:
//   N = boat_x + x·cosψ − y·sinψ
//   E = boat_y + x·sinψ + y·cosψ
// Ej.: ψ = 90° (proa al Este): 5 m a proa → +5 m Este; 2 m a estribor → −2 m Norte (Sur).
Point2D bodyToNed(double boat_x, double boat_y, double yaw, double x_body, double y_body) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Point2D{boat_x + x_body * c - y_body * s,
                 boat_y + x_body * s + y_body * c};
}

ObstacleTracker::ObstacleTracker(double merge_distance, double timeout)
: merge_distance_(merge_distance), timeout_(timeout) {
  if (merge_distance_ < 0.0) {
    throw std::invalid_argument("ObstacleTracker: merge_distance debe ser >= 0");
  }
}

int ObstacleTracker::findMatch(const Detection& d) const {
  if (!d.uuid.empty()) {
    for (size_t k = 0; k < tracks_.size(); ++k) {
      if (tracks_[k].uuid == d.uuid) return static_cast<int>(k);
    }
  }
  int best = -1;
  double best_dist = merge_distance_;
  for (size_t k = 0; k < tracks_.size(); ++k) {
    // Si los dos traen uuid y son distintos, son objetos distintos aunque estén cerca.
    if (!d.uuid.empty() && !tracks_[k].uuid.empty()) continue;
    double dist = std::hypot(tracks_[k].obstacle.x - d.x, tracks_[k].obstacle.y - d.y);
    if (dist <= best_dist) {
      best_dist = dist;
      best = static_cast<int>(k);
    }
  }
  return best;
}

void ObstacleTracker::update(const std::vector<Detection>& detections, double now) {
  for (const auto& d : detections) {
    int k = findMatch(d);
    if (k < 0) {
      tracks_.push_back(Track{Obstacle{d.x, d.y, d.radius}, d.uuid, now});
    } else {
      Track& t = tracks_[k];
      t.obstacle = Obstacle{d.x, d.y, d.radius};  // la detección más reciente manda
      if (!d.uuid.empty()) t.uuid = d.uuid;
      t.last_seen = now;
    }
  }
}

bool ObstacleTracker::prune(double now) {
  if (timeout_ <= 0.0) return false;
  const size_t before = tracks_.size();
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                  [&](const Track& t) { return now - t.last_seen > timeout_; }),
                tracks_.end());
  return tracks_.size() != before;
}

std::vector<Obstacle> ObstacleTracker::obstacles() const {
  std::vector<Obstacle> out;
  out.reserve(tracks_.size());
  for (const auto& t : tracks_) out.push_back(t.obstacle);
  return out;
}

size_t ObstacleTracker::size() const {
  return tracks_.size();
}
