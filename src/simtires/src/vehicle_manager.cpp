#include "simtires/vehicle_manager.hpp"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono/core/ChTypes.h" // for chrono_types::make_shared
#include <algorithm> // for std::clamp

VehicleManager::VehicleManager(std::shared_ptr<rclcpp::Node> node)
: m_node(node), m_next_vehicle_id(0) {
    m_default_vehicle_config = m_node->get_parameter("vehicle_config").as_string();
    RCLCPP_INFO(m_node->get_logger(), "VehicleManager: Using config '%s'", m_default_vehicle_config.c_str());
}

void VehicleManager::initialize(std::shared_ptr<chrono::ChSystem> system) {
    RCLCPP_INFO(m_node->get_logger(), "Initializing vehicle manager...");
    
    m_system = system;
    
    // Create default vehicle
    setupDefaultVehicle();
    
    // Setup services for vehicle management
    m_spawn_service = m_node->create_service<std_srvs::srv::Empty>(
        "spawn_vehicle",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
               std::shared_ptr<std_srvs::srv::Empty::Response> response) {
            (void)request; // Unused
            (void)response; // Unused
            int id = spawnVehicle(m_default_vehicle_config);
            RCLCPP_INFO(m_node->get_logger(), "Spawned vehicle with ID: %d", id);
        });
    
    RCLCPP_INFO(m_node->get_logger(), "Vehicle manager initialized successfully");
}

void VehicleManager::setTerrain(std::shared_ptr<chrono::vehicle::ChTerrain> terrain) {
    m_terrain = terrain;
    
    if (m_terrain) {
        // Set up active domains for existing vehicles
        std::vector<std::shared_ptr<chrono::ChBody>> vehicle_bodies;
        for (const auto& [id, vehicle_instance] : m_vehicles) {
            if (vehicle_instance.vehicle) {
                vehicle_bodies.push_back(vehicle_instance.vehicle->GetChassisBody());
            }
        }
        
        // This would need to be called on the SimulationWorld
        // For now, log that we have the terrain reference
        RCLCPP_INFO(m_node->get_logger(), "Terrain reference set for %zu vehicles", vehicle_bodies.size());
    }
}

void VehicleManager::step() {
    double current_time = m_node->get_clock()->now().seconds();
    double step_size = m_node->get_parameter("step_size").as_double();
    
    // Update all vehicles
    for (auto& [id, vehicle_instance] : m_vehicles) {
        if (vehicle_instance.vehicle) {
            // Get control inputs (for now, zero inputs)
            chrono::vehicle::DriverInputs inputs;
            inputs.m_throttle = vehicle_instance.control_inputs.throttle;
            inputs.m_steering = vehicle_instance.control_inputs.steering;
            inputs.m_braking = vehicle_instance.control_inputs.braking;
            
            // CRITICAL: Synchronize vehicle with terrain
            if (m_terrain) {
                // This is the correct way to synchronize vehicle with SCM terrain
                vehicle_instance.vehicle->Synchronize(current_time, inputs, *m_terrain);
            } else {
                RCLCPP_WARN_ONCE(m_node->get_logger(), "No terrain reference - vehicle will fall through ground");
            }
            
            // Advance vehicle dynamics
            vehicle_instance.vehicle->Advance(step_size);
            
            // Update odometry
            updateVehicleOdometry(vehicle_instance);
        }
    }
}

int VehicleManager::spawnVehicle(
        const std::string& vehicle_config,
        const std::string& namespace_prefix) {
    RCLCPP_INFO(m_node->get_logger(), "Spawning vehicle with config: %s", vehicle_config.c_str());
    
    VehicleInstance instance;
    instance.vehicle_id = m_next_vehicle_id++;
    instance.namespace_prefix = namespace_prefix;
    
    // Create HMMWV vehicle
    instance.vehicle = std::make_shared<chrono::vehicle::hmmwv::HMMWV_Full>(m_system.get());
    instance.vehicle->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    instance.vehicle->SetContactMethod(chrono::ChContactMethod::SMC);
    instance.vehicle->SetChassisFixed(false);
    
    // Set initial position - offset by vehicle ID to avoid overlaps
    chrono::ChVector3d init_pos(-15.0 + instance.vehicle_id * 5.0, -6.0, 2.0);
    instance.vehicle->SetInitPosition(chrono::ChCoordsys<>(init_pos, chrono::QUNIT));
    
    instance.vehicle->SetEngineType(chrono::vehicle::EngineModelType::SHAFTS);
    instance.vehicle->SetTransmissionType(chrono::vehicle::TransmissionModelType::AUTOMATIC_SHAFTS);
    instance.vehicle->SetDriveType(chrono::vehicle::DrivelineTypeWV::AWD);
    instance.vehicle->SetTireType(chrono::vehicle::TireModelType::RIGID);
    
    // Initialize the vehicle - this must be done BEFORE setting visualization
    instance.vehicle->Initialize();
    
    instance.vehicle->SetChassisVisualizationType(chrono::VisualizationType::MESH);
    instance.vehicle->SetSuspensionVisualizationType(chrono::VisualizationType::PRIMITIVES);
    instance.vehicle->SetSteeringVisualizationType(chrono::VisualizationType::PRIMITIVES);
    instance.vehicle->SetTireVisualizationType(chrono::VisualizationType::MESH);
    
    // Create ROS interface for this vehicle
    createVehicleROSInterface(instance);
    
    // Store vehicle
    m_vehicles[instance.vehicle_id] = std::move(instance);
    
    RCLCPP_INFO(m_node->get_logger(), "Vehicle %d spawned successfully", instance.vehicle_id);
    return instance.vehicle_id;
}

bool VehicleManager::removeVehicle(int vehicle_id) {
    auto it = m_vehicles.find(vehicle_id);
    if (it != m_vehicles.end()) {
        m_vehicles.erase(it);
        RCLCPP_INFO(m_node->get_logger(), "Vehicle %d removed", vehicle_id);
        return true;
    }
    RCLCPP_WARN(m_node->get_logger(), "Vehicle %d not found for removal", vehicle_id);
    return false;
}

void VehicleManager::setupDefaultVehicle() {
    // Spawn the first vehicle
    spawnVehicle(m_default_vehicle_config, "");
}

void VehicleManager::createVehicleROSInterface(VehicleInstance& vehicle) {
    std::string topic_prefix = vehicle.namespace_prefix.empty() ? "" : vehicle.namespace_prefix + "/";
    
    // Create command velocity subscriber
    vehicle.cmd_vel_sub = m_node->create_subscription<geometry_msgs::msg::Twist>(
        topic_prefix + "cmd_vel", 10,
        [this, id = vehicle.vehicle_id](const geometry_msgs::msg::Twist::SharedPtr msg) {
            handleVehicleControl(id, msg);
        });
    
    // Create odometry publisher
    vehicle.odom_pub = m_node->create_publisher<nav_msgs::msg::Odometry>(
        topic_prefix + "odom", 10);
    
    RCLCPP_INFO(m_node->get_logger(), "ROS interface created for vehicle %d", vehicle.vehicle_id);
}

void VehicleManager::updateVehicleOdometry(VehicleInstance& vehicle) {
    if (!vehicle.odom_pub) return;
    
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = m_node->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = vehicle.namespace_prefix.empty() ? "base_link" : vehicle.namespace_prefix + "/base_link";
    
    // Get vehicle position and orientation from Chrono
    auto chassis_body = vehicle.vehicle->GetChassisBody();
    auto pos = chassis_body->GetPos();
    auto rot = chassis_body->GetRot();
    
    // Position
    odom_msg.pose.pose.position.x = pos.x();
    odom_msg.pose.pose.position.y = pos.y();
    odom_msg.pose.pose.position.z = pos.z();
    
    // Orientation (quaternion)
    odom_msg.pose.pose.orientation.w = rot.e0();
    odom_msg.pose.pose.orientation.x = rot.e1();
    odom_msg.pose.pose.orientation.y = rot.e2();
    odom_msg.pose.pose.orientation.z = rot.e3();
    
    // Velocity
    auto vel = chassis_body->GetPos();
    auto ang_vel = chassis_body->GetAngVelParent();
    
    odom_msg.twist.twist.linear.x = vel.x();
    odom_msg.twist.twist.linear.y = vel.y();
    odom_msg.twist.twist.linear.z = vel.z();
    
    odom_msg.twist.twist.angular.x = ang_vel.x();
    odom_msg.twist.twist.angular.y = ang_vel.y();
    odom_msg.twist.twist.angular.z = ang_vel.z();
    
    vehicle.odom_pub->publish(odom_msg);
}

void VehicleManager::handleVehicleControl(
        int vehicle_id,
        const geometry_msgs::msg::Twist::SharedPtr msg) {
    auto it = m_vehicles.find(vehicle_id);
    if (it == m_vehicles.end()) {
        RCLCPP_WARN(m_node->get_logger(), "Received control for unknown vehicle %d", vehicle_id);
        return;
    }
    
    // Convert ROS Twist to Chrono DriverInputs
    auto& vehicle = it->second;
    
    // Simple mapping - you can refine this based on your vehicle dynamics
    double max_speed = 10.0; // m/s - adjust based on your requirements
    
    vehicle.control_inputs.throttle = std::clamp(msg->linear.x / max_speed, 0.0, 1.0);
    vehicle.control_inputs.steering = std::clamp(-msg->angular.z, -1.0, 1.0); // Negative for correct direction
    vehicle.control_inputs.braking = 0.0; // Could use negative linear.x for braking
    
    if (msg->linear.x < 0) {
        vehicle.control_inputs.throttle = 0.0;
        vehicle.control_inputs.braking = std::clamp(-msg->linear.x / max_speed, 0.0, 1.0);
    }
    
    RCLCPP_DEBUG(m_node->get_logger(), "Vehicle %d: throttle=%.2f, steering=%.2f, braking=%.2f", 
                vehicle_id, vehicle.control_inputs.throttle, 
                vehicle.control_inputs.steering, vehicle.control_inputs.braking);
}

std::vector<std::shared_ptr<chrono::ChBody>> VehicleManager::getVehicleChassisBodies() const {
    std::vector<std::shared_ptr<chrono::ChBody>> bodies;
    for (const auto& [id, vehicle_instance] : m_vehicles) {
        if (vehicle_instance.vehicle) {
            bodies.push_back(vehicle_instance.vehicle->GetChassisBody());
        }
    }
    return bodies;
}
