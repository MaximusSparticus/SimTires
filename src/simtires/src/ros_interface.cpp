#include "simtires/ros_interface.hpp"
#include "simtires/simulation_world.hpp"
#include "simtires/vehicle_manager.hpp"
#include "simtires/sensor_manager.hpp"

RosInterface::RosInterface(std::shared_ptr<rclcpp::Node> node)
: m_node(node) {
    RCLCPP_INFO(m_node->get_logger(), "RosInterface created");
}

void RosInterface::initialize(
        std::shared_ptr<SimulationWorld> world,
        std::shared_ptr<VehicleManager> vehicle_manager,
        std::shared_ptr<SensorManager> sensor_manager) {
    RCLCPP_INFO(m_node->get_logger(), "Initializing ROS interface...");
    
    m_world = world;
    m_vehicle_manager = vehicle_manager;
    m_sensor_manager = sensor_manager;
    
    // TODO: Setup additional ROS2 topics and services
    // For now, just log that we're ready
    
    RCLCPP_INFO(m_node->get_logger(), "ROS interface initialized successfully");
}

void RosInterface::step() {
    // TODO: Publish TF transforms, visualization markers, etc.
    // For now, this is a placeholder
}
