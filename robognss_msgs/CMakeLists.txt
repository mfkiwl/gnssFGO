cmake_minimum_required(VERSION 3.16)
project(robognss_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-comment)
endif()


if("$ENV{ROS_VERSION}" STREQUAL "1")
    message(STATUS "Compiling RoboGNSS ROS 1 wrapper ...")
    find_package(catkin REQUIRED COMPONENTS
            geometry_msgs
            nav_msgs
            rospy
            sensor_msgs
            std_msgs
    )
    add_message_files(
            FILES
            SatHeader.msg
            Attitude.msg
            Velocity.msg
            GNSSHeader.msg
            PVA.msg
            PVAECEF.msg
            PVT.msg
            PVTECEF.msg
            UTM.msg
            OdomUTM.msg
            EphemBDS.msg
            EphemGPS.msg
            EphemGAL.msg
            EphemGLO.msg
            EphemSBAS.msg
            GNSSRawHeader.msg
            Obs.msg
            ObsExtendedData.msg
            ObsExtended.msg
            MeasEpochRaw.msg
            MeasEpochPreProcessed.msg
            MeasEpochRawMultiAnt.msg
            MeasEpochPreProcessedMultiAnt.msg
            IonUtc.msg
            StationParam.msg
            RTCMData.msg
            RTCM3.msg
            RTCMRaw.msg
            NtripReport.msg
            UWBMeas.msg
            UWB.msg
            EphemGPSArray.msg
            EphemGALArray.msg
            EphemGLOArray.msg
            EphemBDSArray.msg
            EphemSBASArray.msg
    )

    add_service_files(
            FILES
            NtripUpdateServer.srv
    )

    generate_messages(
            DEPENDENCIES
            geometry_msgs  nav_msgs  sensor_msgs  std_msgs
    )

    catkin_package(
            CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
    )
elseif ("$ENV{ROS_VERSION}" STREQUAL "2")
    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)

    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
            msg/SatHeader.msg
            msg/Attitude.msg
            msg/Velocity.msg
            msg/GNSSHeader.msg
            msg/PVA.msg
            msg/PVAECEF.msg
            msg/PVT.msg
            msg/PVTECEF.msg
            msg/UTM.msg
            msg/OdomUTM.msg
            msg/EphemBDS.msg
            msg/EphemGPS.msg
            msg/EphemGAL.msg
            msg/EphemGLO.msg
            msg/EphemSBAS.msg
            msg/GNSSRawHeader.msg
            msg/Obs.msg
            msg/ObsExtended.msg
            msg/MeasEpochRaw.msg
            msg/MeasEpochPreProcessed.msg
            msg/MeasEpochRawMultiAnt.msg
            msg/MeasEpochPreProcessedMultiAnt.msg
            msg/IonUtc.msg
            msg/StationParam.msg
            msg/RTCMData.msg
            msg/RTCM3.msg
            msg/RTCMRaw.msg
            msg/NtripReport.msg
            msg/UWBMeas.msg
            msg/UWB.msg
            msg/EphemGPSArray.msg
            msg/EphemGALArray.msg
            msg/EphemGLOArray.msg
            msg/EphemBDSArray.msg
            msg/EphemSBASArray.msg
            DEPENDENCIES std_msgs geometry_msgs nav_msgs tf2_geometry_msgs
    )

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # uncomment the line when a copyright and license is not present in all source files
        #set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # uncomment the line when this package is not in a git repo
        #set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
    endif()

    ament_export_dependencies(rosidl_default_runtime)
    ament_package()
else()
    set(-DNO_ROS)
    message(FATAL_ERROR "catkin or colcon not found robognss_ros disabled")
endif()




