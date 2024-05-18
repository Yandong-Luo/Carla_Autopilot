#include "planner/route_planner.h"

using namespace autopilot::route_planner;

RoutePlanner::RoutePlanner(Point3D start_point, Point3D end_point,
                 boost::shared_ptr<carla::client::Map> carla_map, double sampling_resolution)
                 : start_point_(std::move(start_point)),
                 end_point_(std::move(end_point)),
                 map_(std::move(carla_map)),
                 sampling_resolution_(sampling_resolution) {
                    
}

void RoutePlanner::BuildPology(){
    topoloy_vec_.clear();
    for(const auto& segment : map_->GetTopology()){
        auto entry_wp = segment.first;
        auto exit_wp = segment.second;
        auto entry_loc = entry_wp->GetTransform().location;
        auto exit_loc = exit_wp->GetTransform().location;
        entry_loc = {round(entry_loc.x), round(entry_loc.y), round(entry_loc.z)};
        exit_loc = {round(exit_loc.x), round(exit_loc.y), round(exit_loc.z)};
        auto entry_3dpoint = Point3D(entry_loc.x, entry_loc.y, entry_loc.z);
        auto exit_3dpoint = Point3D(exit_loc.x, exit_loc.y, exit_loc.z);
        std::vector<cc::Waypoint> pology_path;

        if(entry_loc.Distance(exit_loc) > sampling_resolution_){
            auto next_wp = entry_wp->GetNext(sampling_resolution_)[0];
            while(next_wp->GetTransform().location.Distance(exit_loc) > sampling_resolution_){
                pology_path.emplace_back(next_wp);
                auto next_next_wp = next_wp->GetNext(sampling_resolution_);
                if(next_next_wp.size() == 0)    break;
                next_wp = next_next_wp[0];
            }
        }
        else{
            auto next_wp = entry_wp->GetNext(sampling_resolution_);
            if(next_wp.size() == 0) continue;
            pology_path.emplace_back(next_wp[0]);
        }
        Topology m_topology(entry_wp, exit_wp, pology_path, entry_3dpoint, exit_3dpoint);
        topoloy_vec_.emplace_back(m_topology);
    }
}

// boost::shared_ptr<cc::Waypoint> RoutePlanner::localize(const Point3D& point){
//     auto waypoint = map_->GetWaypoint(cg::Location(point.x_, point.y_, point.z_));
//     auto lane_id = waypoint->GetLaneId();
//     auto road_id = waypoint->GetRoadId();
//     auto section_id = waypoint->GetSectionId();

//     auto edge = road_id_to_edge_[road_id][section_id][lane_id];

//     // auto end_wp = map_->GetWaypoint(cg::Location(end_point_.x_, end_point_.y_, end_point_.z_));

//     return edge;
// }

void RoutePlanner::BuildGraph(){
    graph_ = Graph();
    for(auto segment : topoloy_vec_){
        auto entry_wp = segment.GetEntryWayPoint();
        const uint64_t entry_wp_id = entry_wp->GetId();
        auto exit_wp = segment.GetExitWayPoint();
        const uint64_t exit_wp_id = exit_wp->GetId();

        bool intersection = entry_wp->IsJunction();
        auto road_id = entry_wp->GetRoadId();
        auto section_id = entry_wp->GetSectionId();
        auto lane_id = entry_wp->GetLaneId();
        
        // add node for graph
        graph_.add_node(entry_wp);
        graph_.add_node(exit_wp);

        // auto pair_node = std::make_pair(entry_wp, exit_wp);
        std::pair<const uint64_t, const uint64_t> pair_node_id = std::make_pair<entry_wp_id, exit_wp_id>;
        
        auto entry_vector = entry_wp->GetTransform().rotation.GetForwardVector();
        auto exit_vector = exit_wp->GetTransform().rotation.GetForwardVector();

        // adding edge with attributes
        // graph_.add_edge
        // segment

        graph_.add_edge(pair_node_id, segment.GetPath().size()+1, 
                            carla::traffic_manager::RoadOption::LaneFollow,
                            segment.GetPath());
    } 
}

Topology::Topology(boost::shared_ptr<carla::client::Waypoint> entry_waypoint, 
                    boost::shared_ptr<carla::client::Waypoint> exit_waypoint,
                    std::vector<cc::Waypoint> path, Point3D entry_3Dpoint, Point3D exit_3Dpoint)
                    :entry_waypoint_(std::move(entry_waypoint)),
                    exit_waypoint_(std::move(exit_waypoint)),
                    path_(std::move(path)),
                    entry_3dpoint_(std::move(entry_3Dpoint)),
                    exit_3dpoint_(std::move(exit_3Dpoint)){
}

boost::shared_ptr<cc::Waypoint> Topology::GetEntryWayPoint(){
    return entry_waypoint_;
}

boost::shared_ptr<cc::Waypoint> Topology::GetExitWayPoint(){
    return exit_waypoint_;
}

std::vector<cc::Waypoint> Topology::GetPath(){
    return path_;
}

void Graph::add_node(boost::shared_ptr<carla::client::Waypoint> node){
    if(node_map.find(node->GetId()) == node_map.end()){
        node_map[node->GetId()] = node;
    }    
}

void Graph::add_edge(std::pair<uint64_t, uint64_t> pair_node_id,
                     int length, carla::traffic_manager::RoadOption road_type,
                      std::vector<cc::Waypoint> path){
    // auto new_edge = Edge(edge, length)
    if(node_map.find(pair_node_id.first) == node_map.end()){
        std::cout<<"Could not found node"<<pair_node_id.first<<std::endl;
    }
    auto entry_wp = node_map[pair_node_id.first];
    auto entry_wp_id = entry_wp->GetId();
    auto road_id = entry_wp->GetRoadId();
    auto lane_id = entry_wp->GetLaneId();
    auto section_id = entry_wp->GetSectionId();

    road_id_to_edge_[road_id][section_id][lane_id] = pair_node_id;
    auto new_edge = Edge(pair_node_id, length, road_type, path);

    traj_map[entry_wp_id].emplace_back(new_edge);
}

std::vector<uint64_t> Graph::AStarSolver(const Point3D& start_pos, const Point3D& end_pos, boost::shared_ptr<carla::client::Map> carla_map){
    std::queue<AStar_Node, Node_Comparator> open_set;
    // std::unordered_map<uint64_t, WayPoint_Node> closed_set;
    std::vector<uint64_t> closed_set;
    std::unordered_map<uint64_t, double> node_score;
    std::unordered_map<uint64_t, uint64_t> node_path;

    auto start_wp = carla_map->GetWaypoint(cg::Location(start_pos.x_, start_pos.y_, start_pos.z_));
    auto end_wp = carla_map->GetWaypoint(cg::Location(end_pos.x_, end_pos.y_, end_pos.z_));
    // auto start_wp_id = start_wp->GetId();
    auto graph_source_edge = road_id_to_edge_[start_wp->GetRoadId()][start_wp->GetSectionId()][start_wp->GetLaneId()];
    auto graph_target_edge = road_id_to_edge_[end_wp->GetRoadId()][end_wp->GetSectionId()][end_wp->GetLaneId()];

    // set h to zero, A Star can convert to Dijkstra which guarantee generate the shortest path
    node_score[graph_source_edge.first] = 0.0;
    AStar_Node start_node(graph_source_edge.first, 0.0);
    open_set.push(start_node);

    while(!open_set.empty()){
        auto current_node = open_set.front();
        auto cur_node_id = current_node.id_;
        open_set.pop();
        auto cur_waypoint = node_map[cur_node_id];

        if(GetWayPointDist(current_node, node_map[graph_target_edge.first]) < distance_threshold_){
            std::cout<<"A star found"<<std::endl;
            std::vector<uint64_t> route;

            while(find(closed_set.begin(), closed_set.end(), cur_node_id) != closed_set.end()){
                route.emplace_back(cur_node_id);
                
                cur_node_id = node_path.find(cur_node_id)->second;
                // current_node = 
            }
            std::reverse(route.begin(), route.end());
            return route;
        }

        auto neighbor_nodes = traj_map[cur_node_id];
        for(const auto& neighbor : neighbor_nodes){
            auto next_id = neighbor.edge_.second;
            if(find(closed_set.begin(), closed_set.end(), next_id) != closed_set.end()){
                continue;
            }
            auto next_score = current_node.score_ + neighbor.len;
        }
    }
}

double Graph::GetWayPointDist(const boost::shared_ptr<cc::Waypoint>& p1, const boost::shared_ptr<cc::Waypoint>& p2){
    cg::Location p1_loc = p1->GetTransform().location;
    cg::Location p2_loc = p2->GetTransform().location;
    return std::sqrt(std::pow((p1_loc.x - p2_loc.x),2.0)
                    + std::pow((p1_loc.y - p2_loc.y),2.0)
                    + std::pow((p1_loc.z - p2_loc.z),2.0));
}