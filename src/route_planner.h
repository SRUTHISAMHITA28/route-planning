#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include "route_model.h"

enum class TransportMode {
    DRIVING,
    WALKING,
    CYCLING
};

struct RouteStats {
    float distance;           // in kilometers
    float estimated_time;     // in minutes
    int num_intersections;
    float elevation_gain;     // in meters
    TransportMode mode;
    float average_speed;      // in km/h
};

class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, const std::vector<std::pair<float, float>>& waypoints, TransportMode mode = TransportMode::DRIVING);
    float GetDistance() const {return distance;}
    float GetEstimatedTime() const {return stats.estimated_time;}
    void AStarSearch();
    void OptimizeWaypoints();

    // Public methods for testing
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();

  private:
    std::vector<RouteModel::Node*> open_list_forward;
    std::vector<RouteModel::Node*> open_list_backward;
    std::vector<RouteModel::Node*> waypoint_nodes;
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;
    RouteModel::Node *meeting_node;

    RouteStats stats;
    float distance = 0.0f;
    RouteModel &m_Model;

    void BidirectionalSearch(RouteModel::Node* start, RouteModel::Node* end);
    bool CheckForMeeting();
    void CalculateRouteStats();
    float EstimateTravelTime(float distance, int intersections);
};

#endif