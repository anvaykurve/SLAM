// --- CORE ROS2 LIBRARY ---
// Required for the fundamental ROS2 node structure, logging, time management, and spin functions.
#include <rclcpp/rclcpp.hpp>

// --- SENSOR MESSAGE TYPES ---
// Required to process the angular velocity data coming from the /imu topic.
#include <sensor_msgs/msg/imu.hpp>
// Required to process the wheel speed data coming from the /wheel_speed_avg topic.
#include <std_msgs/msg/float32.hpp>

// --- VISUALIZATION MESSAGE TYPES ---
// Required to ingest the array of perceived cones from the perception pipeline.
#include <visualization_msgs/msg/marker_array.hpp>
// Required to publish the car's predicted pose arrow and the uncertainty ellipse to RViz.
#include <visualization_msgs/msg/marker.hpp>

// --- TRANSFORM MATH ---
// Required to convert the car's Euler yaw angle into a Quaternion for the RViz marker pose.
#include <tf2/LinearMath/Quaternion.h>

// --- LINEAR ALGEBRA ---
// Required for all EKF matrix operations (Jacobians, Covariances, Inverses, Eigenvalue decomposition).
#include <Eigen/Dense>

// --- STANDARD MATH ---
// Required for trigonometric functions (sin, cos, atan2) and square roots used in motion and measurement models.
#include <cmath>

class EKFLocalizationNode : public rclcpp::Node {
public:
    EKFLocalizationNode() : Node("ekf_localization_node"), 
                            current_omega_z_(0.0), first_msg_received_(false) {
        
        // Initialize State Vector: [x, y, yaw]^T
        state_estimate_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Initialize Covariance Matrix (P)
        P_ = Eigen::Matrix3d::Identity() * 0.1;

        // Process Noise Covariance (Q)
        Q_ = Eigen::Matrix3d::Identity() * 0.05;

        // Measurement Noise Covariance (R) - Assuming x, y measurements
        R_ = Eigen::Matrix2d::Identity() * 0.1;

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, 
            std::bind(&EKFLocalizationNode::imuCallback, this, std::placeholders::_1));

        rpm_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel_speed_avg", 10, 
            std::bind(&EKFLocalizationNode::rpmCallback, this, std::placeholders::_1));

        observation_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/perceived_cones", 10,
            std::bind(&EKFLocalizationNode::observationCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car/ekf_pose", 10);
        ellipse_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car/ekf_uncertainty", 10);
        
        RCLCPP_INFO(this->get_logger(), "EKF Localization Node initialized.");
    }

private:
    Eigen::Vector3d state_estimate_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix2d R_;

    double current_omega_z_;
    rclcpp::Time last_time_;
    bool first_msg_received_;

    const double wheel_radius_ = 0.25;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpm_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr observation_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ellipse_pub_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_omega_z_ = msg->angular_velocity.z;
    }

    void rpmCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        rclcpp::Time current_time = this->now();

        if (!first_msg_received_) {
            last_time_ = current_time;
            first_msg_received_ = true;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            last_time_ = current_time;
            return;
        }
        last_time_ = current_time;

        double rpm = msg->data;
        double v = rpm * (2.0 * M_PI / 60.0) * wheel_radius_;
        double v_dt = v * dt;

        // Caching trigonometric functions
        double theta = state_estimate_(2);
        double c_theta = std::cos(theta);
        double s_theta = std::sin(theta);

        // --- STEP 1: MOTION UPDATE (PREDICTION) ---
        
        state_estimate_(0) += v_dt * c_theta;
        state_estimate_(1) += v_dt * s_theta;
        state_estimate_(2) += current_omega_z_ * dt;

        // Jacobian of Motion Model (G)
        Eigen::Matrix3d G;
        G << 1.0, 0.0, -v_dt * s_theta,
             0.0, 1.0,  v_dt * c_theta,
             0.0, 0.0,  1.0;

        // Covariance Prediction
        P_ = G * P_ * G.transpose() + Q_;

        publishPose();
        publishUncertaintyEllipse();
    }

    void observationCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        // --- STEP 2: MEASUREMENT UPDATE (CORRECTION) ---
        
        // Pre-allocate matrices to prevent reallocation inside the loop
        static const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 2, 3> H;
        Eigen::Matrix2d S;
        Eigen::Matrix<double, 3, 2> K;
        Eigen::Vector2d actual_Z, expected_Z, innovation;

        for (const auto& marker : msg->markers) {
            
            // 1. Data Association (Requires implementation from Checkpoint 3)
            double map_x = 0.0; 
            double map_y = 0.0; 
            
            // 2. Expected Measurement
            double delta_x = map_x - state_estimate_(0);
            double delta_y = map_y - state_estimate_(1);
            double q = (delta_x * delta_x) + (delta_y * delta_y);
            
            // Safety shield to prevent division by zero
            if (q < 1e-6) {
                continue; 
            }
            
            double sqrt_q = std::sqrt(q);
            
            // 3. Compute Jacobian of Measurement Model (H)
            H << -(delta_x / sqrt_q), -(delta_y / sqrt_q), 0.0,
                  (delta_y / q),      -(delta_x / q),       -1.0;

            // 4. Compute Kalman Gain (K)
            S = H * P_ * H.transpose() + R_;
            K = P_ * H.transpose() * S.inverse();

            // 5. Compute Innovation (Z - expected_Z)
            actual_Z << marker.pose.position.x, marker.pose.position.y; 
            expected_Z << sqrt_q, std::atan2(delta_y, delta_x) - state_estimate_(2);
            
            innovation = actual_Z - expected_Z;
            
            // Angle normalization for the bearing innovation
            while (innovation(1) > M_PI) innovation(1) -= 2.0 * M_PI;
            while (innovation(1) < -M_PI) innovation(1) += 2.0 * M_PI;

            // 6. State and Covariance Update
            state_estimate_ += K * innovation;
            P_ = (I - K * H) * P_;
        }
    }

    void publishPose() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; 
        marker.header.stamp = this->now();
        marker.ns = "car_pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = state_estimate_(0);
        marker.pose.position.y = state_estimate_(1);
        marker.pose.position.z = 0.5;

        tf2::Quaternion q;
        q.setRPY(0, 0, state_estimate_(2));
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.5; 
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }

    void publishUncertaintyEllipse() {
        visualization_msgs::msg::Marker ellipse;
        ellipse.header.frame_id = "map";
        ellipse.header.stamp = this->now();
        ellipse.ns = "covariance";
        ellipse.id = 1;
        ellipse.type = visualization_msgs::msg::Marker::CYLINDER;
        ellipse.action = visualization_msgs::msg::Marker::ADD;

        ellipse.pose.position.x = state_estimate_(0);
        ellipse.pose.position.y = state_estimate_(1);
        ellipse.pose.position.z = 0.0;

        // Eigenvalue decomposition to determine axis lengths
        Eigen::Matrix2d cov_xy = P_.block<2, 2>(0, 0);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov_xy);

        // Scale by chi-square value for 95% confidence interval (approx 5.991)
        double scale_factor = 5.991; 
        ellipse.scale.x = 2.0 * std::sqrt(scale_factor * eig.eigenvalues()(0));
        ellipse.scale.y = 2.0 * std::sqrt(scale_factor * eig.eigenvalues()(1));
        ellipse.scale.z = 0.01; // Flat cylinder

        double angle = std::atan2(eig.eigenvectors()(1, 0), eig.eigenvectors()(0, 0));
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        ellipse.pose.orientation.x = q.x();
        ellipse.pose.orientation.y = q.y();
        ellipse.pose.orientation.z = q.z();
        ellipse.pose.orientation.w = q.w();

        ellipse.color.r = 0.0f;
        ellipse.color.g = 0.0f;
        ellipse.color.b = 1.0f;
        ellipse.color.a = 0.3; // Semi-transparent

        ellipse_pub_->publish(ellipse);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
