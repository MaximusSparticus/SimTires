#include "simtires/simulation_world.hpp"
#include "simtires/chrono_solver_setup.hpp"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChTypes.h"
#include "chrono/utils/ChOpenMP.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include <cstdlib>
#include <algorithm>

SimulationWorld::SimulationWorld(std::shared_ptr<rclcpp::Node> node)
: m_node(node) {
    // Get parameters
    m_world_config = m_node->get_parameter("world_config").as_string();
    m_step_size = m_node->get_parameter("step_size").as_double();
    
    RCLCPP_INFO(m_node->get_logger(), "SimulationWorld: Using config '%s'", m_world_config.c_str());
}

void SimulationWorld::initialize() {
    RCLCPP_INFO(m_node->get_logger(), "Initializing simulation world...");
    
    // Check data directory - first try environment variable, then compile-time definition
    std::string data_dir;
    if (std::getenv("CHRONO_DATA_DIR")) {
        data_dir = std::string{std::getenv("CHRONO_DATA_DIR")};
    }
    if (data_dir.empty()) {
        data_dir = std::string{CHRONO_DATA_DIR};
    }

    // Make sure it ends with a '/' - this is a requirement of Chrono
    if (data_dir.back() != '/') {
        data_dir += "/";
    }

    // Set the paths
    chrono::SetChronoDataPath(data_dir);
    chrono::vehicle::SetVehicleDataPath(data_dir + "/vehicle/");
    RCLCPP_INFO(m_node->get_logger(), "Chrono Data Directory set to: %s", data_dir.c_str());
    
    // Create Chrono system
    m_system = std::make_shared<chrono::ChSystemSMC>();
    m_system->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    
    // Set gravity
    m_system->SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -9.81));
    
    // Configure threading
    int num_threads_chrono = std::min(8, chrono::ChOMP::GetNumProcs());
    int num_threads_collision = 1;
    int num_threads_eigen = 1;
    m_system->SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);
    
    RCLCPP_INFO(m_node->get_logger(), "System configured with %d threads", num_threads_chrono);
    
    // Setup solver
    auto solver = chrono::ChSolver::Type::BARZILAIBORWEIN;
    auto integrator = chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    simtires::SetupChronoSolver(*m_system, solver, integrator);
    
    setupTerrain();
    
    RCLCPP_INFO(m_node->get_logger(), "Simulation world initialized successfully");
}

void SimulationWorld::step() {
    if (m_system && m_terrain) {
        // CRITICAL: Synchronize terrain BEFORE advancing physics
        // This updates terrain deformation and contact forces
        double current_time = m_node->get_clock()->now().seconds();
        m_terrain->Synchronize(current_time);
        
        // Advance the physics system
        m_system->DoStepDynamics(m_step_size);
    }
}

void SimulationWorld::addActiveDomainsForVehicles(
        const std::vector<std::shared_ptr<chrono::ChBody>>& vehicle_bodies) {
    if (!m_terrain) {
        RCLCPP_WARN(m_node->get_logger(), "Cannot add active domains - terrain not initialized");
        return;
    }
    
    RCLCPP_INFO(m_node->get_logger(), "Adding active domains for %zu vehicles", vehicle_bodies.size());
    
    // Add active domain for each vehicle chassis
    // This tells the SCM terrain to only compute deformation around these bodies
    for (const auto& body : vehicle_bodies) {
        if (body) {
            // Create a box around each vehicle for terrain computation
            // Adjust size based on your vehicle dimensions
            chrono::ChVector3d domain_size(5.0, 3.0, 2.0); // length, width, height
            m_terrain->AddActiveDomain(body, chrono::ChVector3d(0, 0, 0), domain_size);
        }
    }
    
    RCLCPP_INFO(m_node->get_logger(), "Active domains added successfully");
}

void SimulationWorld::setupTerrain() {
    RCLCPP_INFO(m_node->get_logger(), "Setting up terrain with config: %s", m_world_config.c_str());
    
    // Create terrain based on config
    m_terrain = std::make_shared<chrono::vehicle::SCMTerrain>(m_system.get());
    
    // Set soil parameters for SCM terrain
    m_terrain->SetSoilParameters(2e6,   // Bekker Kphi
                                0,      // Bekker Kc  
                                1.1,    // Bekker n exponent
                                0,      // Mohr cohesive limit (Pa)
                                30,     // Mohr friction limit (degrees)
                                0.01,   // Janosi shear coefficient (m)
                                2e8,    // Elastic stiffness (Pa/m)
                                3e4     // Damping (Pa s/m)
    );
    
    // Initialize terrain based on config
    double delta = 0.05;  // Grid resolution
    
    try {
        if (m_world_config == "flat_world") {
            // Flat terrain
            chrono::ChVector2d patch_size(40.0, 16.0);
            m_terrain->Initialize(patch_size.x(), patch_size.y(), delta);
            RCLCPP_INFO(m_node->get_logger(), "Initialized flat terrain: %.1fx%.1f m", patch_size.x(), patch_size.y());
        } else if (m_world_config == "bump_world") {
            // Bump terrain from mesh file
            std::string mesh_file = chrono::vehicle::GetVehicleDataFile("terrain/meshes/bump.obj");
            RCLCPP_INFO(m_node->get_logger(), "Loading terrain mesh from: %s", mesh_file.c_str());
            m_terrain->Initialize(mesh_file, delta);
            RCLCPP_INFO(m_node->get_logger(), "Initialized bump terrain from mesh");
        } else {
            // Default to bump terrain 
            std::string mesh_file = chrono::vehicle::GetVehicleDataFile("terrain/meshes/bump.obj");
            RCLCPP_INFO(m_node->get_logger(), "Unknown config '%s', defaulting to bump terrain", m_world_config.c_str());
            RCLCPP_INFO(m_node->get_logger(), "Loading terrain mesh from: %s", mesh_file.c_str());
            m_terrain->Initialize(mesh_file, delta);
            RCLCPP_INFO(m_node->get_logger(), "Initialized bump terrain from mesh");
        }
        
        // Configure visualization
        m_terrain->GetMesh()->SetWireframe(true);
        
        // Try to set texture - this might fail if texture file not found
        try {
            std::string texture_file = chrono::vehicle::GetVehicleDataFile("terrain/textures/dirt.jpg");
            RCLCPP_INFO(m_node->get_logger(), "Loading terrain texture from: %s", texture_file.c_str());
            m_terrain->GetMesh()->SetTexture(texture_file);
        } catch (const std::exception& e) {
            RCLCPP_WARN(m_node->get_logger(), "Failed to load terrain texture: %s", e.what());
        }
        
        // Set up visualization coloring
        m_terrain->SetColormap(chrono::ChColormap::Type::FAST);
        m_terrain->SetPlotType(chrono::vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.08);
        
        RCLCPP_INFO(m_node->get_logger(), "Terrain setup complete");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(m_node->get_logger(), "Failed to initialize terrain: %s", e.what());
        RCLCPP_ERROR(m_node->get_logger(), "Falling back to flat terrain");
        
        // Fallback to flat terrain
        chrono::ChVector2d patch_size(40.0, 16.0);
        m_terrain->Initialize(patch_size.x(), patch_size.y(), delta);
        m_terrain->GetMesh()->SetWireframe(true);
        RCLCPP_INFO(m_node->get_logger(), "Fallback flat terrain initialized");
    }
}

void SimulationWorld::loadWorldConfig() {
    // TODO: Load world configuration from YAML files
    RCLCPP_INFO(m_node->get_logger(), "Loading world config: %s", m_world_config.c_str());
}
