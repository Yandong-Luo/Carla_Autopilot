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

std::vector<Point3D> Planner::GetRoutePointsByAStar(){
    std::set<WayPoint_Node, WayPoint_Comparator> open_set;
    // record the relationship bewteen parent and child node;
    // first is child, second is the parent
    std::unordered_map<uint64_t, uint64_t> path;
    std::unordered_map<uint64_t, boost::shared_ptr<carla::client::Waypoint>> closed_node;
    std::unordered_map<uint64_t, double> score;

    auto start_watpoint = map_->GetWaypoint(cg::Location(start_point_.x_, start_point_.y_, start_point_.z_));
    auto end_watpoint = map_->GetWaypoint(cg::Location(end_point_.x_, end_point_.y_, end_point_.z_));
    score[start_watpoint->GetId()] = 0.0;
    WayPoint_Node start_node(start_watpoint, GetWayPointDist(start_watpoint, end_watpoint));
    open_set.insert(start_node);

    while(!open_set.empty()){
        auto cur_node = open_set.begin();
        open_set.erase(cur_node);
        auto cur_waypoint = cur_node->GetWayPoint();
        auto cur_waypoint_id = cur_waypoint->GetId();

        if(GetWayPointDist(cur_waypoint, end_watpoint) < distance_threshold_){
            std::cout<<"A Star found"<<std::endl;
            std::vector<Point3D> result;
            std::vector<boost::shared_ptr<cc::Waypoint>> waypoint_result;
            while(closed_node.find(cur_waypoint_id) != closed_node.end()){
                waypoint_result.emplace_back(cur_waypoint);
                auto cur_node_loc = cur_waypoint->GetTransform().location;
                result.emplace_back(Point3D{cur_node_loc.x, cur_node_loc.y, cur_node_loc.z});

                // select the parent as the next node
                cur_waypoint = closed_node[path.find(cur_waypoint_id)->second];
                cur_waypoint_id = cur_waypoint->GetId();
            }
            // reverse the result so that start_point to end point
            std::reverse(result.begin(), result.end());
            std::reverse(waypoint_result.begin(), waypoint_result.end());
            return result;
        }

        // Based on current lane add next waypoint to open_set
        auto next_waypoints = cur_waypoint->GetNext(waypoint_distance_);
        // add next waypoint to open_set from left and right lane
        auto lane_change_flag = cur_waypoint->GetLaneChange();
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
        
        for(const auto& waypoint : next_waypoints){
            auto waypoint_id = waypoint->GetId();
            if(closed_node.find(waypoint_id) != closed_node.end())  continue;
            auto waypoint_score = GetWayPointDist(start_watpoint, waypoint) + GetWayPointDist(end_watpoint, waypoint);
            
            if(score.find(waypoint_id) == score.end() || score[waypoint_id] > waypoint_score){
                score[waypoint_id] = waypoint_score;
                closed_node[cur_waypoint_id] = cur_waypoint;
                WayPoint_Node next_waypoint_node(waypoint, waypoint_score);
                if(open_set.find(next_waypoint_node) != open_set.end()){
                    open_set.erase(next_waypoint_node);
                }
                open_set.insert(next_waypoint_node);
            }
        }
        std::cout << "A* not found" << std::endl;
        std::vector<Point3D> result;
        return result;
    }

}