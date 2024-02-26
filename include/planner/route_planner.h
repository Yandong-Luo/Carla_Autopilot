#ifndef ROUTER_PLANNER_
#define ROUTER_PLANNER_

#include<iostream>
#include<math.h>
#include <set>
#include <carla/client/Map.h>
#include <carla/road/element/LaneMarking.h>
#include <carla/trafficmanager/SimpleWaypoint.h>
#include "utils/utils.h"
#include <queue>
#include <algorithm>

using namespace autopilot::utils;
namespace cc = carla::client;
namespace cg = carla::geom;
namespace cr = carla::road;

namespace autopilot{
    namespace route_planner{
        class Topology{
            private:
            boost::shared_ptr<cc::Waypoint> entry_waypoint_;
            boost::shared_ptr<cc::Waypoint> exit_waypoint_;
            std::vector<cc::Waypoint> path_;
            Point3D entry_3dpoint_;
            Point3D exit_3dpoint_;

            public:
            Topology() = default;
            Topology(boost::shared_ptr<carla::client::Waypoint> entry_waypoint, boost::shared_ptr<carla::client::Waypoint> exit_waypoint,
                     std::vector<cc::Waypoint> path, Point3D entry_3Dpoint, Point3D exit_3Dpoint);
            
            boost::shared_ptr<cc::Waypoint> GetEntryWayPoint();
            boost::shared_ptr<cc::Waypoint> GetExitWayPoint();
            std::vector<cc::Waypoint> GetPath();
            // friend class Graphy;
        };

        class Edge{
            public:
            std::pair<uint64_t, uint64_t> edge_;
            int len;
            carla::traffic_manager::RoadOption road_type_;
            Point3D entry_vector;
            Point3D exit_vector;
            std::vector<cc::Waypoint> edge_path_;
            Edge() = default;
            Edge(std::pair<int, int> edge,
                 int length, carla::traffic_manager::RoadOption road_type, 
                 std::vector<cc::Waypoint> path)
                :edge_(std::move(edge)),
                len(length),
                road_type_(road_type),
                edge_path_(std::move(path))
                {
                    // auto entry_loc = edge_.first->GetTransform().location;
                    // auto exit_loc = edge_.second->GetTransform().location;
                    // auto entry_vector = Point3D(entry_loc.x, entry_loc.y, entry_loc.z);
                    // auto exit_vector = Point3D(exit_loc.x, exit_loc.y, exit_loc.z);
                }
        };

        class Graph{
            private:
            std::unordered_map<uint64_t, boost::shared_ptr<cc::Waypoint>> node_map;
            // std::unordered_map<int, boost::shared_ptr<cc::Waypoint>> id_to_waypoint;
            // std::unordered_map<int, uint64_t> node_id_to_id_;
            // std::vector<uint64_t> node_list;
            std::unordered_map<uint64_t, std::vector<Edge>> traj_map;

            std::unordered_map<cr::RoadId, 
            std::unordered_map<cr::SectionId, 
            std::unordered_map<cr::LaneId, 
            std::pair<uint64_t, uint64_t>>>> road_id_to_edge_;

            double distance_threshold_ = 1.0;

            // std::vector<edge> edge_list;
            public:
            Graph() = default;
            void add_node(boost::shared_ptr<cc::Waypoint> node);
            void add_edge(std::pair<uint64_t, uint64_t> pair_node_id, int length, 
                          carla::traffic_manager::RoadOption road_type, std::vector<cc::Waypoint> path);
            std::vector<uint64_t> AStarSolver(const Point3D& start_pos, const Point3D& end_pos, boost::shared_ptr<carla::client::Map> carla_map);
            double GetWayPointDist(const boost::shared_ptr<cc::Waypoint>& p1, const boost::shared_ptr<cc::Waypoint>& p2);
        };

        struct AStar_Node{
            // private:
            uint64_t id_;
            double score_;
            // public:
            AStar_Node() = default;
            AStar_Node(uint64_t node_id, double score):id_(node_id), score_(score){}
            // uint64_t GetId();
            // double GetScore();
        };
        struct Node_Comparator{
            bool operator()(const AStar_Node& node1, const AStar_Node& node2){
                if(node1.id_ == node2.id_)  return false;
                else{
                    if(node1.score_ == node2.score_){
                        return node1.id_ > node2.id_;
                    }
                    else return node1.score_ < node2.score_;
                }
            }
        };

        class RoutePlanner{
            private:
            boost::shared_ptr<carla::client::Map> map_;
            Point3D start_point_;
            Point3D end_point_;
            double sampling_resolution_ = 1.0;
            std::vector<Topology> topoloy_vec_;
            Graph graph_;
            std::unordered_map<uint64_t, boost::shared_ptr<cc::Waypoint>> node_record;

            public:
            RoutePlanner() = default;
            RoutePlanner(Point3D start_point, Point3D end_point, boost::shared_ptr<cc::Map> carla_map, double sampling_resolution);
            void BuildPology();
            void BuildGraph();

            // boost::shared_ptr<cc::Waypoint> localize(const Point3D& point);
        };

    }
}

#endif