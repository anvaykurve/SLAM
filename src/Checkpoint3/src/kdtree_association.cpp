#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>

struct State { double x, y, yaw; };
struct Reading { double range, bearing; };
struct Landmark { double x, y; int original_index; };

// --- KD-Tree Implementation ---
struct KDNode {
    Landmark data;
    std::unique_ptr<KDNode> left;
    std::unique_ptr<KDNode> right;
    KDNode(Landmark pt) : data(pt), left(nullptr), right(nullptr) {}
};

class KDTree {
private:
    std::unique_ptr<KDNode> root;

    std::unique_ptr<KDNode> buildRecursive(std::vector<Landmark>& points, int start, int end, int depth) {
        if (start >= end) return nullptr;

        int axis = depth % 2; // 0 for X-axis, 1 for Y-axis
        int mid = start + (end - start) / 2;

        std::nth_element(points.begin() + start, points.begin() + mid, points.begin() + end,
            [axis](const Landmark& a, const Landmark& b) {
                return (axis == 0) ? (a.x < b.x) : (a.y < b.y);
            });

        auto node = std::make_unique<KDNode>(points[mid]);
        node->left = buildRecursive(points, start, mid, depth + 1);
        node->right = buildRecursive(points, mid + 1, end, depth + 1);
        return node;
    }

    void nearestRecursive(const KDNode* node, const Landmark& target, int depth, double& best_dist, int& best_idx) const {
        if (!node) return;

        double d = std::hypot(node->data.x - target.x, node->data.y - target.y);
        if (d < best_dist) {
            best_dist = d;
            best_idx = node->data.original_index;
        }

        int axis = depth % 2;
        double axis_dist = (axis == 0) ? (target.x - node->data.x) : (target.y - node->data.y);

        const KDNode* first_path = (axis_dist < 0) ? node->left.get() : node->right.get();
        const KDNode* second_path = (axis_dist < 0) ? node->right.get() : node->left.get();

        nearestRecursive(first_path, target, depth + 1, best_dist, best_idx);

        // Only search the other branch if it's geometrically possible to find a closer point
        if (std::abs(axis_dist) < best_dist) {
            nearestRecursive(second_path, target, depth + 1, best_dist, best_idx);
        }
    }

public:
    KDTree(std::vector<Landmark> points) {
        root = buildRecursive(points, 0, points.size(), 0);
    }

    int findNearest(const Landmark& target, double threshold) const {
        double best_dist = std::numeric_limits<double>::max();
        int best_idx = -1;
        nearestRecursive(root.get(), target, 0, best_dist, best_idx);
        return (best_dist <= threshold) ? best_idx : -1;
    }
};

// --- Task 3.3 Association Function ---
std::vector<int> kdTreeAssociation(
    const State& car_state, 
    const std::vector<Reading>& sensor_readings, 
    const std::vector<Landmark>& known_map) 
{
    // Build the KD-Tree once for the known map
    KDTree tree(known_map);
    
    std::vector<int> associated_indices;
    double threshold = 2.0; 

    for (const auto& reading : sensor_readings) {
        double global_obs_x = car_state.x + reading.range * std::cos(car_state.yaw + reading.bearing);
        double global_obs_y = car_state.y + reading.range * std::sin(car_state.yaw + reading.bearing);

        Landmark target = {global_obs_x, global_obs_y, -1};
        associated_indices.push_back(tree.findNearest(target, threshold));
    }

    return associated_indices;
}

int main() {
    State car = {0.0, 0.0, 0.0}; 

    std::vector<Reading> readings = {
        {5.0, 0.0},     
        {10.198, 0.197},
        {20.0, 0.0}      
    };

    // Note: added the 'original_index' parameter to track map indices after sorting
    std::vector<Landmark> map = {
        {5.0, 0.0, 0},
        {10.0, 2.0, 1},
        {50.0, 50.0, 2}
    };

    std::vector<int> results = kdTreeAssociation(car, readings, map);

    std::cout << "--- Task 3.3 KD-Tree Results ---" << std::endl;
    for (size_t i = 0; i < results.size(); ++i) {
        std::cout << "Sensor Reading " << i << " matched to Map Index: " << results[i] << std::endl;
    }

    return 0;
}
