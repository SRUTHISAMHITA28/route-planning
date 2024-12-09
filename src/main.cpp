#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;

static std::optional<std::vector<std::byte>> ReadFile(const std::string& path) {
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if (!is)
        return std::nullopt;

    auto size = is.tellg();
    std::vector<std::byte> contents(size);

    is.seekg(0);
    is.read((char*)contents.data(), size);

    if (contents.empty())
        return std::nullopt;
    return std::move(contents);
}

static void PrintRouteStats(const RoutePlanner& planner) {
    cout << "\nRoute Statistics:" << endl;
    cout << "=================" << endl;
    cout << "Distance: " << planner.GetDistance() << " km" << endl;
    cout << "Estimated Time: " << planner.GetEstimatedTime() << " minutes" << endl;
    cout << "Transport Mode: ";
    switch(planner.GetStats().mode) {
        case TransportMode::DRIVING: cout << "Driving"; break;
        case TransportMode::CYCLING: cout << "Cycling"; break;
        case TransportMode::WALKING: cout << "Walking"; break;
    }
    cout << endl;
    cout << "Average Speed: " << planner.GetStats().average_speed << " km/h" << endl;
    cout << "Intersections: " << planner.GetStats().num_intersections << endl;
    cout << "Elevation Gain: " << planner.GetStats().elevation_gain << " meters" << endl;
}

int main(int argc, const char **argv) {
    std::string osm_data_file = "";
    if (argc > 1) {
        osm_data_file = argv[1];
    } else {
        std::cout << "Usage: " << argv[0] << " path_to_osm_data" << std::endl;
        return -1;
    }
    
    auto osm_data = ReadFile(osm_data_file);
    if (!osm_data) {
        std::cout << "Failed to read OSM data." << std::endl;
        return -1;
    }
    
    // Build Model.
    RouteModel model{*osm_data};
    
    std::vector<std::pair<float, float>> waypoints;
    float start_x, start_y, end_x, end_y;
    
    cout << "Enter start x coordinate: ";
    cin >> start_x;
    cout << "Enter start y coordinate: ";
    cin >> start_y;
    cout << "Enter destination x coordinate: ";
    cin >> end_x;
    cout << "Enter destination y coordinate: ";
    cin >> end_y;

    int mode_choice;
    cout << "\nSelect transportation mode:" << endl;
    cout << "1. Driving" << endl;
    cout << "2. Cycling" << endl;
    cout << "3. Walking" << endl;
    cout << "Choice: ";
    cin >> mode_choice;

    TransportMode selected_mode;
    switch(mode_choice) {
        case 2: selected_mode = TransportMode::CYCLING; break;
        case 3: selected_mode = TransportMode::WALKING; break;
        default: selected_mode = TransportMode::DRIVING; break;
    }

    waypoints.push_back(std::make_pair(start_x, start_y));
    waypoints.push_back(std::make_pair(end_x, end_y));

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, waypoints, selected_mode};
    route_planner.AStarSearch();
    
    PrintRouteStats(route_planner);
    
    Render render{model};
    
    auto display = io2d::output_surface{400,
                                    400,
                                    io2d::format::argb32,
                                    io2d::scaling::none,
                                    io2d::refresh_style::fixed,
                                    30};
    display.size_change_callback([](io2d::output_surface& surface) {
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback(
        [&](io2d::output_surface& surface) { render.Display(surface); });
    display.begin_show();
    
    return 0;
}
