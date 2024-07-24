//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (haoming.zhang@rwth-aachen.de)
//
//

//
// Created by haoming on 04.05.24.
//

#ifndef ONLINE_FGO_IMUPREINTEGRATOR_H
#define ONLINE_FGO_IMUPREINTEGRATOR_H
#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <irt_nav_msgs/msg/fgo_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "IntegratorBase.h"
#include "InitGyroBias_ert_rtw/InitGyroBias.h"

namespace fgo::integrator {
  class IMUPreIntegrator : public IntegratorBase {
    fgo::data::CircularDataBuffer<fgo::data::IMUMeasurement> imuDataBuffer_;
    fgo::data::State lastOptimizedState_;
    fgo::data::State currentPredState_;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> currentIMUPreintegrator_;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subIMU_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velPub_;
    rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStatePredPub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr userEstimationPub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fgoStatePredNavFixPub_;

    rclcpp::TimerBase::SharedPtr pubUserEstimationTimer_;

    std::atomic_bool isDoingPropagation_ = true;


  };
}
#endif //ONLINE_FGO_IMUPREINTEGRATOR_H
