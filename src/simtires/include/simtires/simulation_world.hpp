// Manages the Chrono world, terrain, and global simulation state

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

// Chrono includes
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

class SimulationWorld {
 public:
    SimulationWorld(std::shared_ptr<rclcpp::Node> node);
    ~SimulationWorld() = default;
    
    void initialize();
    void step();
    
    // Add active domains for vehicle bodies (call after vehicles are created)
    void addActiveDomainsForVehicles(const std::vector<std::shared_ptr<chrono::ChBody>>& vehicle_bodies);
    
    // Getters for other components
    std::shared_ptr<chrono::ChSystem> getChronoSystem() const { return m_system; }
    std::shared_ptr<chrono::vehicle::SCMTerrain> getTerrain() const { return m_terrain; }
    
 private:
    void setupTerrain();
    void loadWorldConfig();
    
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<chrono::ChSystem> m_system;
    std::shared_ptr<chrono::vehicle::SCMTerrain> m_terrain;
    
    // Configuration
    std::string m_world_config;
    double m_step_size;
};
