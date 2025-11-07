#include "simtires/visualization_manager.hpp"
#include "simtires/simulation_world.hpp"
#include "simtires/vehicle_manager.hpp"

#include "chrono/core/ChTypes.h" // for chrono_types::make_shared

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

VisualizationManager::VisualizationManager(std::shared_ptr<rclcpp::Node> node)
: m_node(node)
, m_is_initialized(false) {
    m_enable_visualization = m_node->get_parameter("enable_visualization").as_bool();
    RCLCPP_INFO(m_node->get_logger(), "VisualizationManager created (enabled: %s)", 
               m_enable_visualization ? "true" : "false");
}

void VisualizationManager::initialize(
        std::shared_ptr<SimulationWorld> world, 
        std::shared_ptr<VehicleManager> vehicle_manager) {
    RCLCPP_INFO(m_node->get_logger(), "Initializing visualization manager...");
    
    m_world = world;
    m_vehicle_manager = vehicle_manager;
    
    if (m_enable_visualization) {
        setupIrrlichtVisualization();
    } else {
        RCLCPP_INFO(m_node->get_logger(), "Visualization disabled by parameter");
    }
    
    m_is_initialized = true;
    RCLCPP_INFO(m_node->get_logger(), "Visualization manager initialized successfully");
}

void VisualizationManager::step() {
    if (!m_enable_visualization || !m_is_initialized) {
        return;
    }
    
    if (m_vis_system) {
        if (!m_vis_system->Run()) {
            RCLCPP_WARN(m_node->get_logger(), "Visualization window closed");
            return;
        }
        
        m_vis_system->BeginScene();
        m_vis_system->Render();
        m_vis_system->EndScene();
        m_vis_system->Advance(m_node->get_parameter("step_size").as_double());
    }
}

bool VisualizationManager::isRunning() const {
    if (m_vis_system) {
        return m_vis_system->Run();
    }
    return true; // If no visualization system, assume still running
}

void VisualizationManager::setupIrrlichtVisualization() {
    RCLCPP_INFO(m_node->get_logger(), "Setting up Irrlicht visualization...");
    
    try {
        // Create Irrlicht visualization system
        m_vis_system = chrono_types::make_shared<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>();
        
        // Configure visualization
        m_vis_system->SetWindowTitle("SimTires - Vehicle Simulation on Deformable Terrain");
        m_vis_system->SetWindowSize(1280, 720);
        m_vis_system->SetChaseCamera(chrono::ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
        
        // Initialize the system
        m_vis_system->Initialize();
        
        // Add lighting and environment
        m_vis_system->AddLightDirectional();
        m_vis_system->AddSkyBox();
        m_vis_system->AddLogo();
        
        // Attach terrain
        if (m_world && m_world->getTerrain()) {
            m_vis_system->AttachTerrain(m_world->getTerrain().get());
            RCLCPP_INFO(m_node->get_logger(), "Terrain attached to visualization");
        }
        
        // Attach vehicles
        if (m_vehicle_manager) {
            const auto& vehicles = m_vehicle_manager->getVehicles();
            for (const auto& [id, vehicle_instance] : vehicles) {
                if (vehicle_instance.vehicle) {
                    m_vis_system->AttachVehicle(&vehicle_instance.vehicle->GetVehicle());
                    RCLCPP_INFO(m_node->get_logger(), "Vehicle %d attached to visualization", id);
                }
            }
        }
        
        RCLCPP_INFO(m_node->get_logger(), "Irrlicht visualization setup complete");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(m_node->get_logger(), "Failed to setup Irrlicht visualization: %s", e.what());
        m_vis_system.reset();
    }
}
