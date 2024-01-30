#ifndef PLANNER_H_
#define PLANNER_H_

#include<iostream>
#include<math.h>
#include <set>
#include <carla/client/Map.h>
#include <carla/road/element/LaneMarking.h>
#include"utils/utils.h"

using namespace autopilot::utils;
namespace cc = carla::client;
namespace cg=carla::geom;

// using namespace carla::client::Map;
namespace autopilot{
    namespace planner{
        class Planner{
            private:
            Point3D start_point_;
            Point3D end_point_;
            boost::shared_ptr<carla::client::Map> map_;
            double distance_threshold_{5.0};
            double waypoint_distance_{1.0};
            public:
            Planner(Point3D start_point, Point3D end_point, boost::shared_ptr<cc::Map> carla_map);
            // std::vector<Point3D> GetRoutePoints();
            std::vector<Point3D> GetRoutePointsByAStar();
            double GetWayPointDist(const boost::shared_ptr<cc::Waypoint>& p1, const boost::shared_ptr<cc::Waypoint>& p2);
            ~Planner();
        };
    }
}

#endif