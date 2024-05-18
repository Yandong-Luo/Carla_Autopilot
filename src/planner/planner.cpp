#include "planner/planner.h"

// using namespace autopilot::utils;
using namespace autopilot::planner;

Planner::Planner(Point3D start_point, Point3D end_point,
                 boost::shared_ptr<carla::client::Map> carla_map)
                 : start_point_(std::move(start_point)),
                 end_point_(std::move(end_point)),
                 map_(std::move(carla_map)) {
}

double Planner::GetWayPointDist(const boost::shared_ptr<cc::Waypoint>& p1, const boost::shared_ptr<cc::Waypoint>& p2){
    cg::Location p1_loc = p1->GetTransform().location;
    cg::Location p2_loc = p2->GetTransform().location;
    return std::sqrt(std::pow((p1_loc.x - p2_loc.x),2.0)
                    + std::pow((p1_loc.y - p2_loc.y),2.0)
                    + std::pow((p1_loc.z - p2_loc.z),2.0));
}

std::vector<carla::geom::Location> Planner::GetRoutePointsByAStar(){
    std::set<WayPoint_Node, WayPoint_Comparator> open_set;
    // record the relationship bewteen parent and child node;
    // first is child, second is the parent
    std::unordered_map<uint64_t, uint64_t> path;
    // std::unordered_map<uint64_t, boost::shared_ptr<carla::client::Waypoint>> wp_map;
    std::unordered_map<uint64_t, boost::shared_ptr<cc::Waypoint>> wp_map;
    // std::unordered_map<uint64_t, uint64_t> closed_node;
    // std::vector<uint64_t> closed_list;
    std::unordered_map<uint64_t, double> score;

    auto start_watpoint = map_->GetWaypoint(cg::Location(start_point_.x_, start_point_.y_, start_point_.z_));
    auto end_watpoint = map_->GetWaypoint(cg::Location(end_point_.x_, end_point_.y_, end_point_.z_));
    score[start_watpoint->GetId()] = 0.0;
    wp_map[start_watpoint->GetId()] = start_watpoint;
    WayPoint_Node start_node(start_watpoint, GetWayPointDist(start_watpoint, end_watpoint));
    open_set.insert(start_node);
    std::cout<<"start openset checking "<<std::endl;
    while(!open_set.empty()){
        auto cur_node_it = open_set.begin();
        auto cur_waypoint = cur_node_it->GetWayPoint();
        auto tmp = cur_waypoint->GetLaneId();
        std::cout<<tmp<<std::endl;
        open_set.erase(cur_node_it);
        auto cur_waypoint_id = cur_waypoint->GetId();
        // std::cout<<"determine reach or not"<<std::endl;
        if(GetWayPointDist(cur_waypoint, end_watpoint) < distance_threshold_){
            std::cout<<"A Star found"<<std::endl;
            std::vector<Point3D> result;
            std::vector<carla::geom::Location> location_result;
            std::vector<boost::shared_ptr<cc::Waypoint>> waypoint_result;
            while(path.find(cur_waypoint_id) != path.end()){
                waypoint_result.emplace_back(cur_waypoint);
                auto cur_node_loc = cur_waypoint->GetTransform().location;
                location_result.emplace_back(cur_node_loc);
                result.emplace_back(Point3D{cur_node_loc.x, cur_node_loc.y, cur_node_loc.z});

                // select the parent as the next node
                cur_waypoint = wp_map[path.find(cur_waypoint_id)->second];
                cur_waypoint_id = cur_waypoint->GetId();
            }
            // reverse the result so that start_point to end point
            std::reverse(result.begin(), result.end());
            std::reverse(waypoint_result.begin(), waypoint_result.end());
            return location_result;
        }

        // Based on current lane add next waypoint to open_set
        auto next_waypoints = cur_waypoint->GetNext(waypoint_distance_);
        // std::cout<<"the number of next waypoints:"<<next_waypoints.size()<<std::endl;
        // add next waypoint to open_set from left and right lane
        auto lane_change_flag = cur_waypoint->GetLaneChange();
        // std::cout<<"add neightbor "<<std::endl;
        if(lane_change_flag == carla::road::element::LaneMarking::LaneChange::Right || 
            lane_change_flag == carla::road::element::LaneMarking::LaneChange::Both){
                auto right_tmp_point = cur_waypoint->GetRight();
                if(right_tmp_point->GetType() == carla::road::Lane::LaneType::Driving){
                    auto right_next_points = right_tmp_point->GetNext(waypoint_distance_);
                    next_waypoints.insert(next_waypoints.end(), right_next_points.begin(), right_next_points.end());
                }
        }
        if(lane_change_flag == carla::road::element::LaneMarking::LaneChange::Left || 
            lane_change_flag == carla::road::element::LaneMarking::LaneChange::Both){
                auto left_tmp_point = cur_waypoint->GetLeft();
                if(left_tmp_point->GetType() == carla::road::Lane::LaneType::Driving){
                    auto left_next_points = left_tmp_point->GetNext(waypoint_distance_);
                    next_waypoints.insert(next_waypoints.end(), left_next_points.begin(), left_next_points.end());
                }
        }
        
        for(const auto& next_waypoint : next_waypoints){
            // std::cout<<"check neightbor "<<std::endl;
            auto next_waypoint_id = next_waypoint->GetId();
            if(path.find(next_waypoint_id) != path.end())  continue;
            auto waypoint_score = score[cur_waypoint_id] + GetWayPointDist(cur_waypoint, next_waypoint);
            
            if(score.find(next_waypoint_id) == score.end() || score[next_waypoint_id] > waypoint_score){
                score[next_waypoint_id] = waypoint_score;
                // auto tmp = boost::make_shared<cc::Waypoint>();
                // wp_map.insert(std::make_pair(next_waypoint_id, next_waypoint));
                // tmp = next_waypoint;
                wp_map[next_waypoint_id] =  next_waypoint;
                // wp_map[next_waypoint_id].reset(new cc::Waypoint(next_waypoint));
                // <cc::Waypoint>(next_waypoint);
                path[next_waypoint_id] = cur_waypoint_id;
                // path[next_waypoint_id] = std::make_shared<cc::Waypoint>(cur_waypoint_id);
                WayPoint_Node next_waypoint_node(next_waypoint, waypoint_score);
                auto next_node_it = open_set.find(next_waypoint_node);
                if(next_node_it != open_set.end()){
                    open_set.erase(next_node_it);
                }
                open_set.insert(next_waypoint_node);
            }
        }
    }
    std::cout << "A* not found" << std::endl;
    // std::vector<Point3D> result;
    std::vector<carla::geom::Location> result;
    return result;
}