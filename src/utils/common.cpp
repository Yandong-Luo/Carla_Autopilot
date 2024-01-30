#include"utils/utils.h"
using namespace autopilot::utils;

// Point3d
bool Point3D::operator==(const Point3D& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0)) < 0.001;
}

double Point3D::Distance(const Point3D& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0));
}

boost::shared_ptr<carla::client::Waypoint> WayPoint_Node::GetWayPoint() const{
  return waypoint_;
}

double WayPoint_Node::GetDistance() const{
  return distance_;
}