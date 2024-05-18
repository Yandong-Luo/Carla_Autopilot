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
            // boost::shared_ptr<carla::client::Waypoint> waypoint_{new carla::SharedPtr<carla::client::Waypoint>};
            double distance_ {std::numeric_limits<double>::max()};
        public:
            WayPoint_Node() = default;
            WayPoint_Node(boost::shared_ptr<carla::client::Waypoint> waypoint, double distance):waypoint_(std::move(waypoint)), distance_(distance){
            }
            // bool comparator(const WayPoint_Node& node1, const WayPoint_Node& node2);
            boost::shared_ptr<carla::client::Waypoint> GetWayPoint() const;
            double GetDistance() const;
            // ~WayPoint_Node();
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

        struct VehicleState 
        {
            double x;
            double y;
            double heading;   // 车辆朝向
            double kappa;     // 曲率(切线斜率)
            double velocity;    // 速度
            double angular_velocity;  // 角速度
            double acceleration;    // 加速度

            // 规划起点
            double planning_init_x; 
            double planning_init_y;

            double roll;  
            double pitch;
            double yaw;

            double target_curv;  // 期望点的曲率

            double vx;
            double vy;
            double vz;
            double ax;
            double ay;
            double az;

            double v;

            // added
            double start_point_x;
            double start_point_y;

            double relative_x = 0;
            double relative_y = 0;

            double relative_distance = 0;

            double start_heading;
        };

        struct PathProfile{
            std::vector<double>* headings;
            std::vector<double>* accumulated_s;
            std::vector<double>* kappas;
            std::vector<double>* dkappas;
        };

        bool ComputePathProfile(const std::vector<carla::geom::Location> route_wypoint,
                                std::vector<double>* headings,
                                std::vector<double>* accumulated_s,
                                std::vector<double>* kappas,
                                std::vector<double>* dkappas);
                                
        VehicleState UpdateVehicleState();
    }
}

#endif