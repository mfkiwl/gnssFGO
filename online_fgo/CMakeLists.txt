cmake_minimum_required(VERSION 3.16...3.26)
project(online_fgo VERSION 0.3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Wno-unused-parameter -Wno-return-type)
endif ()

#set(CMAKE_CXX_FLAGS_RELEASE "-g -pthread")

# path for custom find scripts, use same of libRSF
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()


# find ROS2 dependencies
find_package(NumCpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(irt_nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(message_filters REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb tbbmalloc)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(keyboard_handler REQUIRED)

# LIOSAM 
find_package(lio_sam REQUIRED)
find_package(OpenCV REQUIRED)

# find other dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3.5 REQUIRED NO_MODULE)

# GTSAM
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(GTSAMCMakeTools)

include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)
# include_directories(${GTSAM_INCLUDE_DIR}) 

# Ceres to solve the NLS problem
#find_package(Ceres 2.0 REQUIRED) # ToDo: actually we don't need ceres, but i just put it here

# Simple multi-threading,
set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
# find_package(Threads REQUIRED)

# geographic lib for coordinate frame conversions
find_package(GeographicLib REQUIRED)

# rosbag2 utility
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)

# for unittest scripts, only if we need
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# **************************************************************************** #
# ************************** ONLINE FGO SETUP ******************************** #
set(ONLINEFGO_PREFIX
        online_fgo
)
# ************************** INCLUDE & LINK ********************************** #
add_subdirectory(third_party) # used by ${tools_lib_list}
set(ONLINEFGO_ROS_DEP
        "rclcpp"
        "rclcpp_components"
        "rclcpp_lifecycle"
        "std_msgs"
        "sensor_msgs"
        "irt_nav_msgs"
        "message_filters"
        "pluginlib"
        "nav_msgs"
        "PCL"
        "novatel_oem7_msgs"
        "ublox_msgs"
        "rosbag2_cpp"
        "rosbag2_storage"
        "keyboard_handler"
        "pcl_msgs"
)
set(ONLINEFGO_GENERAL_INCLUDE
        ${GTSAM_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
        /usr/include/eigen3
        ${PCL_INCLUDE_DIRS}
)
set(ONLINEFGO_GENERAL_LINK
        ${GeographicLib_LIBRARIES}
        gtsam
        gtsam_unstable
        Threads::Threads
        Eigen3::Eigen
        pthread
        tbb
        tbbmalloc
        ${PCL_LIBRARIES}
        ${OpenMP_CXX_FLAGS}
)
set(ONLINEFGO_INCLUDE
        ${PROJECT_SOURCE_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/third_party
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${ONLINEFGO_GENERAL_INCLUDE}
)
set(ONLINEFGO_LINK
        ${tools_lib_list}
        rosbag2_cpp::rosbag2_cpp
        rosbag2_storage::rosbag2_storage
        ${ONLINEFGO_GENERAL_LINK}
)
set(ONLINEFGO_BUILD_TARGET
) # it is empty and target name will be collected by subdirs
add_compile_options(${OpenMP_CXX_FLAGS})

# ************************** Subdirectory ********************************** #
# subdirs are inter-dependent, change the order with caution
add_subdirectory(src/utils)
add_subdirectory(src/sensor)
add_subdirectory(src/solver)
add_subdirectory(src/integrator)
add_subdirectory(src/data)
add_subdirectory(src/graph)
add_subdirectory(src/gnss_fgo)
add_subdirectory(src/offline_process)


# ************************** Nodes ********************************** #

set(node_dep_ros  #ToDo @haoming check and clean this
        "rclcpp"
        "rclcpp_components"
        "rclcpp_lifecycle"
        "cv_bridge"
        "yaml_cpp_vendor"
        "rosidl_default_generators"
        "std_msgs"
        "sensor_msgs"
        "irt_nav_msgs"
        "pluginlib"
        "rmw"
        "rmw_implementation_cmake"
        "rmw_implementation"
        "rosbag2_cpp"
)
#rclcpp_components_register_nodes(${ONLINEFGO_PREFIX}_gnssfgo "online_fgo::OnlineFGOGNSSINSIntegrationNode")
add_subdirectory(src/node)

# ************************** Install ********************************** #

pluginlib_export_plugin_description_file(${PROJECT_NAME} integrator_ros_plugins.xml)
ament_export_dependencies(${node_dep_ros})
ament_export_include_directories(include/${PROJECT_NAME} include)
ament_export_libraries(${ONLINEFGO_BUILD_TARGET})


# ## Mark executables and/or libraries for installation
install(TARGETS ${ONLINEFGO_BUILD_TARGET}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS ${ONLINEFGO_BUILD_TARGET}
        DESTINATION lib
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.launch")
install(FILES integrator_ros_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
