// Manages Chrono visualization system for vehicles and terrain

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

// Forward declarations
class SimulationWorld;
class VehicleManager;

class VisualizationManager {
 public:
    VisualizationManager(std::shared_ptr<rclcpp::Node> node);
    ~VisualizationManager() = default;
    
    void initialize(std::shared_ptr<SimulationWorld> world, 
                   std::shared_ptr<VehicleManager> vehicle_manager);
    void step();
    
    bool isRunning() const;
    
 private:
    void setupIrrlichtVisualization();
    
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<SimulationWorld> m_world;
    std::shared_ptr<VehicleManager> m_vehicle_manager;
    
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> m_vis_system;
    
    bool m_enable_visualization;
    bool m_is_initialized;
};
