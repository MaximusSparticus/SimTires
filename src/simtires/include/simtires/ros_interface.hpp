// Manages ROS2 topics, services, and TF broadcasts

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

// Forward declarations
class SimulationWorld;
class VehicleManager;
class SensorManager;

class RosInterface {
 public:
    RosInterface(std::shared_ptr<rclcpp::Node> node);
    ~RosInterface() = default;
    
    void initialize(std::shared_ptr<SimulationWorld> world,
                   std::shared_ptr<VehicleManager> vehicle_manager,
                   std::shared_ptr<SensorManager> sensor_manager);
    void step();
    
 private:
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<SimulationWorld> m_world;
    std::shared_ptr<VehicleManager> m_vehicle_manager;
    std::shared_ptr<SensorManager> m_sensor_manager;
};
