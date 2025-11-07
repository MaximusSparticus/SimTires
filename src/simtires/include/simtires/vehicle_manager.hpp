// Manages multiple vehicles, their spawning/removal, and control inputs

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include <memory>
#include <unordered_map>

// Chrono includes
#include "chrono/physics/ChSystem.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/ChTerrain.h"

namespace simtires_srvs {
    // TODO(zmd): Define services
}

struct ControlInputs {
    double throttle = 0.0;  // [0, 1]
    double steering = 0.0;  // [-1, 1]
    double braking = 0.0;   // [0, 1]
};

struct VehicleInstance {
    std::shared_ptr<chrono::vehicle::hmmwv::HMMWV_Full> vehicle;
    rclcpp::Subscription<chrono_ros_interface::msg::DriverInputs>::SharedPtr driver_inputs_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::string namespace_prefix;
    int vehicle_id;
    ControlInputs control_inputs;  // Store current control inputs
};

class VehicleManager {
 public:
    VehicleManager(std::shared_ptr<rclcpp::Node> node);
    ~VehicleManager() = default;
    
    void initialize(std::shared_ptr<chrono::ChSystem> system);
    void setTerrain(std::shared_ptr<chrono::vehicle::ChTerrain> terrain);
    void step();
    
    // Vehicle management
    int spawnVehicle(const std::string& vehicle_config, const std::string& namespace_prefix = "");
    bool removeVehicle(int vehicle_id);
    
    // Access for sensor manager
    const std::unordered_map<int, VehicleInstance>& getVehicles() const { return m_vehicles; }
    
    // Get vehicle chassis bodies for terrain active domains
    std::vector<std::shared_ptr<chrono::ChBody>> getVehicleChassisBodies() const;
    
 private:
    void setupDefaultVehicle();
    void createVehicleROSInterface(VehicleInstance& vehicle);
    void updateVehicleOdometry(VehicleInstance& vehicle);
    void handleVehicleControl(int vehicle_id, const chrono_ros_interface::msg::DriverInputs::SharedPtr msg);
    
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<chrono::ChSystem> m_system;
    std::shared_ptr<chrono::vehicle::ChTerrain> m_terrain;
    
    std::unordered_map<int, VehicleInstance> m_vehicles;
    int m_next_vehicle_id;
    
    // Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_spawn_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_remove_service;
    
    // Configuration
    std::string m_default_vehicle_config;
};
