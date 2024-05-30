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

#ifndef ONLINE_FGO_DATSETDELOCO_H
#define ONLINE_FGO_DATSETDELOCO_H
#pragma once

#include "Dataset.h"

namespace fgo::dataset {
  using namespace fgo::data;

  struct DatasetDELoco : DatasetBase<PVASolution> {
    DataBlock<GNSSMeasurement> data_gnss;
    integrator::param::IntegratorGNSSTCParamsPtr gnss_param_ptr;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_pvt_;

    explicit DatasetDELoco(
        rclcpp::Node &node,
        const std::shared_ptr<DatasetParam> &param,
        integrator::param::IntegratorGNSSTCParamsPtr gnss_param)
        : DatasetBase("DELoco", param),
          data_gnss("GNSS", 0., GNSSDataTimeGetter),
          gnss_param_ptr(gnss_param) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": preparing dataset ...");
      pub_pvt_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/gt_nav_fix",
                                                                    rclcpp::SystemDefaultsQoS());
      auto func_on_gt = [this](const std::vector<PVASolution> &data) -> void {
        for (const auto &pva: data) {
          sensor_msgs::msg::NavSatFix navfix;
          navfix.header.stamp = pva.timestamp;
          navfix.latitude = pva.llh[0] * constants::rad2deg;
          navfix.longitude = pva.llh[1] * constants::rad2deg;
          navfix.altitude = pva.llh[2];
          pub_pvt_->publish(navfix);
        }
      };
      data_reference.setCbOnData(func_on_gt);
      parseDataFromBag();
    }

    void parseDataFromBag() override {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": Parsing raw data into data blocks ...");
      const auto raw_imu_msg = readROSMessages<sensor_msgs::msg::Imu>("/imu/data");
      DataBlock<IMUMeasurement>::DataMap imu_map;
      for (const auto &time_msg_pair: raw_imu_msg) {
        imu_map.insert(
            std::make_pair(time_msg_pair.first, msg2IMUMeasurement(time_msg_pair.second, time_msg_pair.first)));
      }
      data_imu.setData(imu_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": parsed IMU data ...");

      const auto raw_pva_msg = readROSMessages<irt_nav_msgs::msg::PVAGeodetic>("/irt_gnss_preprocessing/PVA");
      DataBlock<PVASolution>::DataMap gps_map;
      DataBlock<State>::DataMap gt_state_map;
      for (const auto &time_msg_pair: raw_pva_msg) {
        auto [pva, state] = sensor::GNSS::parseIRTPVAMsg(time_msg_pair.second, gnss_param_ptr);
        gps_map.insert(std::make_pair(time_msg_pair.first, pva));
        gt_state_map.insert(std::make_pair(time_msg_pair.first, state));
      }
      data_reference_state.setData(gt_state_map, true);
      data_reference.setData(gps_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": parsed GNSS PVA data ...");

      const auto raw_gnss = readROSMessages<irt_nav_msgs::msg::GNSSObsPreProcessed>(
          "/irt_gnss_preprocessing/gnss_obs_preprocessed");
      DataBlock<GNSSMeasurement>::DataMap gnss_map;
      for (const auto &time_msg_pair: raw_gnss) {
        gnss_map.insert(std::make_pair(time_msg_pair.first,
                                       sensor::GNSS::convertGNSSObservationMsg(time_msg_pair.second, gnss_param_ptr)));
      }
      data_gnss.setData(gnss_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": parsed GNSS data ...");
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
      data_gnss.trimData(timestamp_start, timestamp_end);
    }

    data::State referenceToState(const PVASolution &reference) override {
      return sensor::GNSS::PVASolutionToState(reference, gnss_param_ptr->transIMUToAnt1);
    }

    GNSSMeasurement getNextGNSSObservation() { return data_gnss.getNextData(); }

    std::vector<GNSSMeasurement>
    getGNSSObservationBetween(rclcpp::Time &lastOptStateTime, const rclcpp::Time &currentOptStateTime) {
      return data_gnss.getDataBetween(lastOptStateTime, currentOptStateTime);
    }

    std::vector<GNSSMeasurement>
    getGNSSObservationBefore(double timestamp,
                             bool erase = false) {
      return data_gnss.getDataBefore(rclcpp::Time(timestamp * 1e9, RCL_ROS_TIME), erase);
    }
  };
}
#endif //ONLINE_FGO_DATSETDELOCO_H
