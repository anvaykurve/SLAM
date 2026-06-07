#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

struct State { double x, y, yaw; };
struct Reading { double range, bearing; };
struct Landmark { double x, y; };

// Task 3.2: Maximum Likelihood Data Association
std::vector<int> maximumLikelihoodAssociation(
    const State& car_state, 
    const std::vector<Reading>& sensor_readings, 
    const std::vector<Landmark>& known_map,
    const Eigen::Matrix3d& P,
    const Eigen::Matrix2d& R) 
{
    std::vector<int> associated_indices;
    
    // Chi-square threshold for 2 degrees of freedom at 95% confidence
    double chi_square_threshold = 5.991; 

    for (const auto& reading : sensor_readings) {
        int best_idx = -1;
        double min_mahalanobis_dist = std::numeric_limits<double>::max();

        Eigen::Vector2d actual_Z(reading.range, reading.bearing);

        for (size_t i = 0; i < known_map.size(); ++i) {
            double delta_x = known_map[i].x - car_state.x;
            double delta_y = known_map[i].y - car_state.y;
            double q = (delta_x * delta_x) + (delta_y * delta_y);
            
            if (q < 1e-6) continue;
            double sqrt_q = std::sqrt(q);

            // 1. Expected Measurement
            Eigen::Vector2d expected_Z(sqrt_q, std::atan2(delta_y, delta_x) - car_state.yaw);

            // 2. Innovation (Error)
            Eigen::Vector2d innovation = actual_Z - expected_Z;
            while (innovation(1) > M_PI) innovation(1) -= 2.0 * M_PI;
            while (innovation(1) < -M_PI) innovation(1) += 2.0 * M_PI;

            // 3. Jacobian (H)
            Eigen::Matrix<double, 2, 3> H;
            H << -(delta_x / sqrt_q), -(delta_y / sqrt_q), 0.0,
                  (delta_y / q),      -(delta_x / q),       -1.0;

            // 4. Innovation Covariance (S)
            Eigen::Matrix2d S = H * P * H.transpose() + R;

            // 5. Mahalanobis Distance (D^2)
            double mahalanobis_dist = innovation.transpose() * S.inverse() * innovation;

            if (mahalanobis_dist < min_mahalanobis_dist) {
                min_mahalanobis_dist = mahalanobis_dist;
                best_idx = static_cast<int>(i);
            }
        }

        // 6. Individual Compatibility Check
        if (min_mahalanobis_dist <= chi_square_threshold) {
            associated_indices.push_back(best_idx);
        } else {
            associated_indices.push_back(-1); 
        }
    }

    return associated_indices;
}

int main() {
    State car = {0.0, 0.0, 0.0}; 

    std::vector<Reading> readings = {
        {5.0, 0.0},      // Perfect match for Map Index 0
        {10.0, 0.5},     // High bearing error, tests Mahalanobis scaling
        {20.0, 0.0}      // Ghost reading
    };

    std::vector<Landmark> map = {
        {5.0, 0.0},
        {10.0, 2.0},
        {50.0, 50.0}
    };

    // Simulate high positional uncertainty but low heading uncertainty
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
    P(0,0) = 2.0; P(1,1) = 2.0; P(2,2) = 0.01; 

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.1;

    std::vector<int> results = maximumLikelihoodAssociation(car, readings, map, P, R);

    std::cout << "--- Task 3.2 Maximum Likelihood Results ---" << std::endl;
    for (size_t i = 0; i < results.size(); ++i) {
        std::cout << "Sensor Reading " << i << " matched to Map Index: " << results[i] << std::endl;
    }

    return 0;
}
