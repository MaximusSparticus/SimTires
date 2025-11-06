# ChronoTargets.cmake
# Creates proper imported targets for Chrono libraries
# Call this after find_package(Chrono)

if(TARGET Chrono::Engine)
    # Targets already created
    return()
endif()

message(STATUS "Chrono library directory: ${CHRONO_LIB_DIR}")
set(CHRONO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/chrono/lib)
set(CHRONO_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/install/chrono/include
    ${CMAKE_CURRENT_SOURCE_DIR}/install/chrono/include/chrono_thirdparty/HACD
    /usr/include/irrlicht
    
)

# Function to create an imported target
function(add_chrono_imported_target target_name lib_name)
    if(NOT TARGET ${target_name})
        add_library(${target_name} SHARED IMPORTED)
        
        # Find Release library
        find_library(${lib_name}_LIBRARY_RELEASE
            NAMES ${lib_name}
            PATHS ${CHRONO_LIB_DIR}
            NO_DEFAULT_PATH
        )
        
        # Find Debug library (with _d suffix)
        find_library(${lib_name}_LIBRARY_DEBUG
            NAMES ${lib_name}_d ${lib_name}
            PATHS ${CHRONO_LIB_DIR}
            NO_DEFAULT_PATH
        )
        
        # Determine which library to use as fallback
        if(${lib_name}_LIBRARY_RELEASE)
            set(FALLBACK_LIB "${${lib_name}_LIBRARY_RELEASE}")
        elseif(${lib_name}_LIBRARY_DEBUG)
            set(FALLBACK_LIB "${${lib_name}_LIBRARY_DEBUG}")
        else()
            message(FATAL_ERROR "Could not find library ${lib_name} in ${CHRONO_LIB_DIR}")
        endif()
        
        # For multi-config generators, set all configurations
        # Use debug version if available, otherwise use release as fallback
        if(${lib_name}_LIBRARY_DEBUG)
            set_target_properties(${target_name} PROPERTIES
                IMPORTED_LOCATION_DEBUG "${${lib_name}_LIBRARY_DEBUG}"
            )
            message(STATUS "  Found ${target_name} (Debug): ${${lib_name}_LIBRARY_DEBUG}")
        else()
            set_target_properties(${target_name} PROPERTIES
                IMPORTED_LOCATION_DEBUG "${FALLBACK_LIB}"
            )
            message(STATUS "  ${target_name} (Debug): using ${FALLBACK_LIB} (no debug version found)")
        endif()
        
        # Use release version if available, otherwise use debug as fallback
        if(${lib_name}_LIBRARY_RELEASE)
            set_target_properties(${target_name} PROPERTIES
                IMPORTED_LOCATION_RELEASE "${${lib_name}_LIBRARY_RELEASE}"
                IMPORTED_LOCATION_RELWITHDEBINFO "${${lib_name}_LIBRARY_RELEASE}"
                IMPORTED_LOCATION_MINSIZEREL "${${lib_name}_LIBRARY_RELEASE}"
            )
            message(STATUS "  Found ${target_name} (Release): ${${lib_name}_LIBRARY_RELEASE}")
        else()
            set_target_properties(${target_name} PROPERTIES
                IMPORTED_LOCATION_RELEASE "${FALLBACK_LIB}"
                IMPORTED_LOCATION_RELWITHDEBINFO "${FALLBACK_LIB}"
                IMPORTED_LOCATION_MINSIZEREL "${FALLBACK_LIB}"
            )
            message(STATUS "  ${target_name} (Release): using ${FALLBACK_LIB} (no release version found)")
        endif()
        
        # Set a default IMPORTED_LOCATION for single-config generators
        set_target_properties(${target_name} PROPERTIES
            IMPORTED_LOCATION "${FALLBACK_LIB}"
        )
        
        # Set include directories
        set_target_properties(${target_name} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${CHRONO_INCLUDE_DIRS}"
        )
        
        # Set compile options if defined
        if(DEFINED CHRONO_CXX_FLAGS)
            set_target_properties(${target_name} PROPERTIES
                INTERFACE_COMPILE_OPTIONS "${CHRONO_CXX_FLAGS}"
            )
        endif()
        
        mark_as_advanced(${lib_name}_LIBRARY_RELEASE ${lib_name}_LIBRARY_DEBUG)
    endif()
endfunction()

# Create core Engine target
add_chrono_imported_target(Chrono::Engine Chrono_core)

# Create Models targets
add_chrono_imported_target(Chrono::Models_Robot ChronoModels_robot)
add_chrono_imported_target(Chrono::Models_Vehicle ChronoModels_vehicle)

# Create module targets with dependencies
macro(add_chrono_module_target target_name lib_name)
    add_chrono_imported_target(${target_name} ${lib_name})
    if(TARGET ${target_name})
        set_property(TARGET ${target_name} APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES Chrono::Engine
        )
    endif()
endmacro()

add_chrono_module_target(Chrono::Vehicle Chrono_vehicle)
add_chrono_module_target(Chrono::Sensor Chrono_sensor)
add_chrono_module_target(Chrono::ROS Chrono_ros)
add_chrono_module_target(Chrono::Parsers Chrono_parsers)
add_chrono_module_target(Chrono::Irrlicht Chrono_irrlicht)
add_chrono_module_target(Chrono::Vehicle_Irrlicht Chrono_vehicle_irrlicht)

# Add Vehicle dependency on Models
if(TARGET Chrono::Vehicle AND TARGET Chrono::Models_Vehicle)
    set_property(TARGET Chrono::Vehicle APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES Chrono::Models_Vehicle Chrono::Models_Robot
    )
endif()

# Add data directory properly
set(CHRONO_SHARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/install/chrono/share/chrono/data")
set_target_properties(Chrono::Engine PROPERTIES INSTALL_RPATH ${CHRONO_SHARE_DIR})
#set_target_properties(Chrono::Vehicle PROPERTIES
#  INTERFACE_COMPILE_DEFINITIONS "CHRONO_VEHICLE_DATA_DIR=\"${CHRONO_SHARE_DIR}/data/vehicle/\""
#  #INTERFACE_INCLUDE_DIRECTORIES "${CHRONO_SHARE_DIR}/include/chrono_vehicle"
#  #INTERFACE_LINK_LIBRARIES "Chrono::Chrono_core"
#)


message(STATUS "Chrono imported targets created successfully")
