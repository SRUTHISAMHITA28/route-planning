# Enhanced Route Planning Project

A C++ route planning application that finds the optimal path between two points on a map using the A* search algorithm. This enhanced version includes support for multiple transportation modes and provides detailed route statistics.

<img src="map.png" width="600" height="450" alt="Route Planning Map"/>

## Features

- Multiple transportation modes:
  - Driving
  - Cycling
  - Walking
- Detailed route statistics:
  - Distance calculation
  - Estimated travel time
  - Number of intersections
  - Elevation gain
  - Average speed based on transport mode
- Interactive command-line interface
- OpenStreetMap data support

## Building the Project

### Prerequisites
- CMake >= 3.11.3
- C++ compiler with C++17 support
- IO2D library

### Build Instructions

1. Clone this repository
2. Create a build directory:
   ```bash
   mkdir build && cd build
   ```
3. Generate build files:
   ```bash
   cmake ..
   ```
4. Build the project:
   ```bash
   cmake --build .
   ```

## Usage

1. Run the program with an OSM data file:
   ```bash
   ./route_planner path_to_osm_data.osm
   ```
2. Enter the start coordinates (x,y)
3. Enter the destination coordinates (x,y)
4. Choose your transportation mode:
   - 1: Driving
   - 2: Cycling
   - 3: Walking
5. The program will calculate and display the optimal route with detailed statistics

## Project Structure

- `src/`: Source files
  - `main.cpp`: Main application entry point
  - `route_planner.h/cpp`: Core route planning logic
  - `route_model.h/cpp`: Data model for the route planner
  - `render.h/cpp`: Visualization components
- `cmake/`: CMake configuration files
- `map.osm`: Sample OpenStreetMap data
- `map.png`: Map visualization
