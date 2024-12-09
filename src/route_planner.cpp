#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, const std::vector<std::pair<float, float>>& waypoints, TransportMode mode)
    : m_Model(model) {
    
    // Set transportation mode
    stats.mode = mode;
    
    // Set default speeds based on transport mode
    switch(mode) {
        case TransportMode::DRIVING:
            stats.average_speed = 50.0f;  // 50 km/h average
            break;
        case TransportMode::CYCLING:
            stats.average_speed = 15.0f;  // 15 km/h average
            break;
        case TransportMode::WALKING:
            stats.average_speed = 5.0f;   // 5 km/h average
            break;
    }

    // Convert waypoints to nodes
    for(const auto& waypoint : waypoints) {
        float x = waypoint.first * 0.01;
        float y = waypoint.second * 0.01;
        waypoint_nodes.push_back(&m_Model.FindClosestNode(x, y));
    }
    
    if (!waypoint_nodes.empty()) {
        start_node = waypoint_nodes.front();
        end_node = waypoint_nodes.back();
    }
    
    stats.distance = 0.0f;
    stats.estimated_time = 0.0f;
    stats.num_intersections = 0;
    stats.elevation_gain = 0.0f;
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->parent = current_node;
            neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->visited = true;
            open_list_forward.push_back(neighbor);
        }
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list_forward.begin(), open_list_forward.end(),
        [](const auto& a, const auto& b) {
            return (a->g_value + a->h_value) > (b->g_value + b->h_value);
        });
    
    auto next = open_list_forward.back();
    open_list_forward.pop_back();
    return next;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    
    while (current_node != nullptr) {
        path_found.push_back(*current_node);
        if (current_node->parent != nullptr) {
            distance += current_node->distance(*current_node->parent);
            stats.num_intersections++;
        }
        current_node = current_node->parent;
    }
    
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale();
    stats.distance = distance;
    CalculateRouteStats();
    
    return path_found;
}

void RoutePlanner::BidirectionalSearch(RouteModel::Node* start, RouteModel::Node* end) {
    start->visited = true;
    end->visited = true;
    
    open_list_forward.push_back(start);
    open_list_backward.push_back(end);
    
    while (!open_list_forward.empty() && !open_list_backward.empty()) {
        if (CheckForMeeting()) {
            break;
        }
        
        auto current_forward = NextNode();
        AddNeighbors(current_forward);
        
        auto current_backward = NextNode();
        AddNeighbors(current_backward);
    }
}

bool RoutePlanner::CheckForMeeting() {
    for (const auto& forward_node : open_list_forward) {
        for (const auto& backward_node : open_list_backward) {
            if (forward_node == backward_node) {
                meeting_node = forward_node;
                return true;
            }
        }
    }
    return false;
}

void RoutePlanner::OptimizeWaypoints() {
    if (waypoint_nodes.size() <= 2) return;
    
    std::vector<RouteModel::Node*> optimized;
    optimized.push_back(waypoint_nodes[0]);
    
    std::vector<bool> visited(waypoint_nodes.size(), false);
    visited[0] = true;
    
    while (optimized.size() < waypoint_nodes.size()) {
        float min_distance = std::numeric_limits<float>::max();
        int next_index = -1;
        
        for (size_t i = 0; i < waypoint_nodes.size(); i++) {
            if (!visited[i]) {
                float dist = optimized.back()->distance(*waypoint_nodes[i]);
                if (dist < min_distance) {
                    min_distance = dist;
                    next_index = i;
                }
            }
        }
        
        if (next_index != -1) {
            optimized.push_back(waypoint_nodes[next_index]);
            visited[next_index] = true;
        }
    }
    
    waypoint_nodes = optimized;
}

float RoutePlanner::EstimateTravelTime(float distance, int intersections) {
    const float avg_speed = 40.0f; // km/h
    const float intersection_delay = 0.5f; // minutes
    
    float time = (distance / 1000.0f) / avg_speed * 60.0f; // Convert to minutes
    time += intersections * intersection_delay;
    
    return time;
}

void RoutePlanner::CalculateRouteStats() {
    stats.estimated_time = EstimateTravelTime(stats.distance, stats.num_intersections);
}

void RoutePlanner::AStarSearch() {
    if (waypoint_nodes.empty()) return;
    
    OptimizeWaypoints();
    
    std::vector<RouteModel::Node> final_path;
    
    for (size_t i = 0; i < waypoint_nodes.size() - 1; i++) {
        BidirectionalSearch(waypoint_nodes[i], waypoint_nodes[i + 1]);
        
        auto segment = ConstructFinalPath(meeting_node);
        final_path.insert(final_path.end(), segment.begin(), segment.end());
        
        // Reset for next segment
        for (auto& node : m_Model.path_found) {
            node->visited = false;
        }
        open_list_forward.clear();
        open_list_backward.clear();
    }
    
    m_Model.path_found = final_path;
}