#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class MotionUpdateNode : public rclcpp::Node {
public:
    MotionUpdateNode() : Node("motion_update_node"), 
                         state_x_(0.0), state_y_(0.0), state_yaw_(0.0), 
                         current_omega_z_(0.0), first_msg_received_(false) {
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, 
            std::bind(&MotionUpdateNode::imuCallback, this, std::placeholders::_1));

        rpm_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel_speed_avg", 10, 
            std::bind(&MotionUpdateNode::rpmCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car/predicted_pose", 10);
        
        RCLCPP_INFO(this->get_logger(), "Motion Update Node initialized and waiting for data...");
    }

private:
    double state_x_, state_y_, state_yaw_;
    double current_omega_z_;
    rclcpp::Time last_time_;
    bool first_msg_received_;

    const double wheel_radius_ = 0.25; // meters

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpm_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_omega_z_ = msg->angular_velocity.z;
    }

    void rpmCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (!first_msg_received_) {
            last_time_ = this->now();
            first_msg_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First sensor message received. Starting calculations.");
            return;
            
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        // Handle rosbag loops or small time jumps
        if (dt <= 0 || dt > 1.0) {
            last_time_ = current_time;
            return;
        }

        last_time_ = current_time;

        // v = omega * r (Assuming msg->data is wheel angular velocity in rad/s)
       double v = msg->data * (2.0 * M_PI / 60.0) * wheel_radius_;

        // Odometry Equations
        state_x_ += v * cos(state_yaw_) * dt;
        state_y_ += v * sin(state_yaw_) * dt;
        state_yaw_ += current_omega_z_ * dt;

        publishMarker();
        
    }

    void publishMarker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; 
        marker.header.stamp = this->now();
        marker.ns = "car_pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = state_x_;
        marker.pose.position.y = state_y_;
        marker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, state_yaw_);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 1.5; 
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
        
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionUpdateNode>());
    rclcpp::shutdown();
    return 0;
}
