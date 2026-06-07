#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

// The structures requested by the packet
struct State { double x, y, yaw; };
struct Reading { double range, bearing; };
struct Landmark { double x, y; };

// Task 3.1: Nearest Neighbors Data Association
std::vector<int> nearestNeighborAssociation(
    const State& car_state, 
    const std::vector<Reading>& sensor_readings, 
    const std::vector<Landmark>& known_map) 
{
    std::vector<int> associated_indices;
    double threshold = 2.0; // 2 meters max distance

    for (const auto& reading : sensor_readings) {
        // --- Project the reading to global coordinates ---
        double global_obs_x = car_state.x + reading.range * std::cos(car_state.yaw + reading.bearing);
        double global_obs_y = car_state.y + reading.range * std::sin(car_state.yaw + reading.bearing);

        int best_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        // --- Loop through the known map to find the closest landmark ---
        for (size_t i = 0; i < known_map.size(); ++i) {
            double dist = std::hypot(known_map[i].x - global_obs_x, known_map[i].y - global_obs_y);
            if (dist < min_dist) {
                min_dist = dist;
                best_idx = static_cast<int>(i);
            }
        }

        // --- Check against the threshold ---
        if (min_dist <= threshold) {
            associated_indices.push_back(best_idx);
        } else {
            associated_indices.push_back(-1); // -1 indicates a new or unmatched landmark
        }
    }

    return associated_indices;
}

// Quick test to verify the math
int main() {
    // Test State: Car is at the origin, facing perfectly forward (yaw = 0)
    State car = {0.0, 0.0, 0.0}; 

    // Test Readings: 
    // Cone 1: 5m straight ahead
    // Cone 2: ~10.2m ahead and slightly left
    // Cone 3: 20m straight ahead (simulating a ghost reading)
    std::vector<Reading> readings = {
        {5.0, 0.0}, 
        {10.198, 0.197},
        {20.0, 0.0}
    };

    // Test Map: One cone at (5, 0), one at (10, 2), one way off at (50, 50)
    std::vector<Landmark> map = {
        {5.0, 0.0},
        {10.0, 2.0},
        {50.0, 50.0}
    };

    // Run the function
    std::vector<int> results = nearestNeighborAssociation(car, readings, map);

    // Print the results
    std::cout << "--- Data Association Results ---" << std::endl;
    for (size_t i = 0; i < results.size(); ++i) {
        std::cout << "Sensor Reading " << i << " matched to Map Index: " << results[i] << std::endl;
    }

    return 0;
}
