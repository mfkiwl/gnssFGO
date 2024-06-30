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
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//
//

//
// Created by haoming on 07.05.24.
//

#ifndef ONLINE_FGO_DATASETBOREAS_H
#define ONLINE_FGO_DATASETBOREAS_H
#pragma once

#include "Dataset.h"

namespace fgo::dataset {
  using namespace fgo::data;

  struct DatasetBoreas : DatasetBase<PVASolution> {
    DataBlock<PVASolution> data_pva_gps;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_{};
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gt_{};
    integrator::param::IntegratorBaseParamsPtr gnss_param_ptr;

    explicit DatasetBoreas(rclcpp::Node &node,
                           const std::shared_ptr<DatasetParam> &param,
                           integrator::param::IntegratorBaseParamsPtr gnss_integrator_param)
        : DatasetBase("Boreas", param),
          data_pva_gps("PVAGNSS", 0., PVADataTimeGetter), gnss_param_ptr(gnss_integrator_param) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": preparing dataset ...");
      pub_gnss_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/boreas/gps_nav_fix",
                                                                     rclcpp::SystemDefaultsQoS());
      pub_gt_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/boreas/gps_gt_fix", rclcpp::SystemDefaultsQoS());

      auto func_on_gt = [this](const std::vector<PVASolution> &data) -> void {
        for (const auto &pva: data) {
          sensor_msgs::msg::NavSatFix navfix;
          navfix.header.stamp = pva.timestamp;
          navfix.latitude = pva.llh[0] * constants::rad2deg;
          navfix.longitude = pva.llh[1] * constants::rad2deg;
          navfix.altitude = pva.llh[2];
          pub_gt_->publish(navfix);
        }
      };

      auto func_on_gnss = [this](const std::vector<PVASolution> &data) -> void {
        for (const auto &pva: data) {
          sensor_msgs::msg::NavSatFix navfix;
          navfix.header.stamp = pva.timestamp;
          navfix.latitude = pva.llh[0] * constants::rad2deg;
          navfix.longitude = pva.llh[1] * constants::rad2deg;
          navfix.altitude = pva.llh[2];
          pub_gnss_->publish(navfix);
        }
      };
      data_reference.setCbOnData(func_on_gt);
      data_pva_gps.setCbOnData(func_on_gnss);
      parseDataFromBag();
    }

    void parseDataFromBag() override {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": Parsing raw data into data blocks ...");
      const auto raw_imu_msg = readROSMessages<sensor_msgs::msg::Imu>("/boreas/imu/data");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got imu");
      const auto raw_gps_pva_msg = readROSMessages<nav_msgs::msg::Odometry>("/boreas/gps_raw");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got odometry");
      const auto raw_gt_pva_msg = readROSMessages<nav_msgs::msg::Odometry>("/boreas/gps_gt");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got odometry");

      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": start converting ...");
      DataBlock<IMUMeasurement>::DataMap imu_map;
      std::cout << gnss_param_ptr->imuRot << std::endl;
      for (const auto &time_msg_pair: raw_imu_msg) {
        imu_map.insert(std::make_pair(time_msg_pair.first, msg2IMUMeasurement(time_msg_pair.second, time_msg_pair.first,
                                                                              gnss_param_ptr->imuRot.matrix())));
      }
      data_imu.setData(imu_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": parsed IMU data ...");

      DataBlock<PVASolution>::DataMap gps_map;
      for (const auto &time_msg_pair: raw_gps_pva_msg) {
        auto [pva, state] = sensor::GNSS::parseOdomMsg(time_msg_pair.second, gnss_param_ptr, time_msg_pair.first);
        gps_map.insert(std::make_pair(time_msg_pair.first, pva));
      }
      data_pva_gps.setData(gps_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": parsed GNSS PVA data ...");

      DataBlock<PVASolution>::DataMap gt_gps_map;
      DataBlock<State>::DataMap gt_state_map;
      for (const auto &time_msg_pair: raw_gt_pva_msg) {
        auto [pva, state] = sensor::GNSS::parseOdomMsg(time_msg_pair.second, gnss_param_ptr, time_msg_pair.first);
        gt_gps_map.insert(std::make_pair(time_msg_pair.first, pva));
        gt_state_map.insert(std::make_pair(time_msg_pair.first, state));
      }
      data_reference_state.setData(gt_state_map, true);
      data_reference.setData(gt_gps_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": parsed Reference GNSS data ...");
      setTimestampsFromReference();
      trimDataBlocks();
    }

    void trimDataBlocks() override {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": trimming datablocks to start timestamp "
                                               << std::fixed << timestamp_start.seconds() << " and end timestamp "
                                               << timestamp_end.seconds());
      data_imu.trimData(timestamp_start, timestamp_end);
      data_reference_state.trimData(timestamp_start, timestamp_end);
      data_reference.trimData(timestamp_start, timestamp_end);
      data_pva_gps.trimData(timestamp_start, timestamp_end);
    }

    data::State referenceToState(const PVASolution &reference) override {
      return sensor::GNSS::PVASolutionToState(reference, gnss_param_ptr->transIMUToAnt1);
    }

    PVASolution getNextGNSSPVA() { return data_pva_gps.getNextData(); }

    std::vector<PVASolution>
    getGNSSPVADataBetween(rclcpp::Time &lastOptStateTime, const rclcpp::Time &currentOptStateTime) {
      return data_pva_gps.getDataBetween(lastOptStateTime, currentOptStateTime);
    }

    std::vector<PVASolution>
    getGNSSPVABefore(double timestamp,
                     bool erase = false) {
      return data_pva_gps.getDataBefore(rclcpp::Time(timestamp * 1e9, RCL_ROS_TIME), erase);
    }

  };


}
#endif //ONLINE_FGO_DATASETBOREAS_H