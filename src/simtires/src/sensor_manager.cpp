#include "simtires/sensor_manager.hpp"
#include "simtires/vehicle_manager.hpp"

SensorManager::SensorManager(std::shared_ptr<rclcpp::Node> node)
: m_node(node) {
    RCLCPP_INFO(m_node->get_logger(), "SensorManager created");
}

void SensorManager::initialize(std::shared_ptr<VehicleManager> vehicle_manager) {
    RCLCPP_INFO(m_node->get_logger(), "Initializing sensor manager...");
    
    m_vehicle_manager = vehicle_manager;
    
    // TODO: Initialize sensors for each vehicle
    // For now, just log that we're ready
    
    RCLCPP_INFO(m_node->get_logger(), "Sensor manager initialized successfully");
}

void SensorManager::step() {
    // TODO: Update sensors and publish sensor data
    // For now, this is a placeholder
}
