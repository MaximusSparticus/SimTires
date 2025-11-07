// Main ROS2 node entrypoint

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "simtires/simulation_world.hpp"
#include "simtires/vehicle_manager.hpp" 
#include "simtires/sensor_manager.hpp"
#include "simtires/ros_interface.hpp"
#include "simtires/visualization_manager.hpp"

class SimulationNode : public rclcpp::Node {
 public:
    SimulationNode() : Node("simtires_simulation") {
        RCLCPP_INFO(get_logger(), "Initializing SimTires simulation...");
        
        // Declare parameters
        declare_parameter("world_config", "flat_world");
        declare_parameter("vehicle_config", "hmmwv_full_sensors");
        declare_parameter("step_size", 0.003);
        declare_parameter("real_time_factor", 1.0);
        declare_parameter("enable_visualization", true);
        
        RCLCPP_INFO(get_logger(), "Parameters declared, ready for initialization");
    }
    
    void initialize() {
        RCLCPP_INFO(get_logger(), "Starting component initialization...");
        
        // Now we can safely use shared_from_this()
        m_world = std::make_shared<SimulationWorld>(shared_from_this());
        m_vehicle_manager = std::make_shared<VehicleManager>(shared_from_this());
        m_sensor_manager = std::make_shared<SensorManager>(shared_from_this());
        m_ros_interface = std::make_shared<RosInterface>(shared_from_this());
        m_vis_manager = std::make_shared<VisualizationManager>(shared_from_this());
        
        // Initialize components in correct order
        m_world->initialize();
        m_vehicle_manager->initialize(m_world->getChronoSystem());
        
        // CRITICAL: Connect terrain to vehicle manager for proper physics interaction
        m_vehicle_manager->setTerrain(m_world->getTerrain());
        
        // Set up active domains for SCM terrain efficiency
        auto vehicle_bodies = m_vehicle_manager->getVehicleChassisBodies();
        m_world->addActiveDomainsForVehicles(vehicle_bodies);
        
        // Initialize remaining components
        m_sensor_manager->initialize(m_vehicle_manager);
        m_ros_interface->initialize(m_world, m_vehicle_manager, m_sensor_manager);
        m_vis_manager->initialize(m_world, m_vehicle_manager);
        
        // Start simulation timer
        double step_size = get_parameter("step_size").as_double();
        m_timer = create_wall_timer(
            std::chrono::duration<double>(step_size),
            std::bind(&SimulationNode::simulationStep, this)
        );
        
        RCLCPP_INFO(get_logger(), "SimTires simulation initialized successfully");
    }

 private:
    void simulationStep() {
        // Update all simulation components
        m_world->step();
        m_vehicle_manager->step();
        m_sensor_manager->step();
        m_ros_interface->step();
        m_vis_manager->step();
    }
    
    // Simulation components
    std::shared_ptr<SimulationWorld> m_world;
    std::shared_ptr<VehicleManager> m_vehicle_manager;
    std::shared_ptr<SensorManager> m_sensor_manager;
    std::shared_ptr<RosInterface> m_ros_interface;
    std::shared_ptr<VisualizationManager> m_vis_manager;
    
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimulationNode>();
    
    try {
        node->initialize();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Simulation error: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
