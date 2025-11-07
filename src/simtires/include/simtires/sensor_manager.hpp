// Manages sensors interfaces (lidar, EO, IMU, GPS, tach, etc)

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

class VehicleManager;

class SensorManager {
public:
    SensorManager(std::shared_ptr<rclcpp::Node> node);
    ~SensorManager() = default;
    
    void initialize(std::shared_ptr<VehicleManager> system);
    void step();
    
private:
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<VehicleManager> m_vehicle_manager;
};

