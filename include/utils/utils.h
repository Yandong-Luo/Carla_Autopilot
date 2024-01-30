#ifndef UTILS_H_
#define UTILS_H_

#include<iostream>
#include<math.h>
#include<carla/client/Waypoint.h>

namespace autopilot{
    namespace utils{
        class Point3D
        {
        private:
            /* data */
        public:
            Point3D() = default;
            Point3D(float x, float y, float z) : x_(x), y_(y), z_(z){}
            float x_{0.0};
            float y_{0.0};
            float z_{0.0};

            bool operator==(const Point3D& p) const;
            double Distance(const Point3D& p) const;
        };
        
        // for route planner by A Star
        class WayPoint_Node
        {
        private:
            boost::shared_ptr<carla::client::Waypoint> waypoint_{nullptr};
            double distance_ = 0.0;
        public:
            WayPoint_Node() = default;
            WayPoint_Node(boost::shared_ptr<carla::client::Waypoint> waypoint, double distance):waypoint_(std::move(waypoint_)), distance_(distance){}
            bool comparator(const WayPoint_Node& node1, const WayPoint_Node& node2);
            boost::shared_ptr<carla::client::Waypoint> GetWayPoint() const;
            double GetDistance() const;
            ~WayPoint_Node();
        };
        
        struct WayPoint_Comparator{
            bool operator()(const WayPoint_Node& wp1, const WayPoint_Node& wp2){
                if(wp1.GetWayPoint()->GetId() == wp2.GetWayPoint()->GetId())    return false;
                else{
                    if(wp1.GetDistance() == wp2.GetDistance()){
                        return wp1.GetWayPoint()->GetId() > wp2.GetWayPoint()->GetId();
                    }
                    else    return wp1.GetDistance() < wp2.GetDistance();
                }
            }
        };
    }
}

#endif